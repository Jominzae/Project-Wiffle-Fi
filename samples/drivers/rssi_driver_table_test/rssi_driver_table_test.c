#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/minmax.h>

#define DEVICE_NAME "rssi_driver_table_test"
#define CLASS_NAME  "rssi_class"

#define MAX_NETWORKS 30
#define MAX_SSID_LEN 32

// read 응답 버퍼 크기(넉넉히)
#define RESP_BUF_SIZE 4096
// write 시 한 번에 받을 최대 길이(너무 큰 write 방지)
#define WRITE_MAX     2048

struct wifi_info {
    char ssid[MAX_SSID_LEN];
    int  rssi;
    bool active;
};

static struct wifi_info rssi_table[MAX_NETWORKS];

static char line_buffer[256];
static int  line_ptr;

static int major_num;
static struct class  *rssi_class;
static struct device *rssi_device;

static DEFINE_MUTEX(table_lock);

/* "SSID:xxx,RSSI:-55" 같은 한 줄을 파싱해서 테이블 갱신 */
static void parse_and_update(const char *line)
{
    const char *s_ptr;
    const char *r_ptr;
    char ssid_tmp[MAX_SSID_LEN];
    int rssi_tmp;
    int len;
    int i;

    s_ptr = strstr(line, "SSID:");
    r_ptr = strstr(line, "RSSI:");

    if (!s_ptr || !r_ptr)
        return;

    /* RSSI 파싱 */
    rssi_tmp = 0;
    if (sscanf(r_ptr + 5, "%d", &rssi_tmp) != 1) {
        printk(KERN_INFO "RSSI_DRV: RSSI parse failed: %s\n", line);
        return;
    }

    /* SSID 파싱: "SSID:" 뒤에서 ',' 또는 '\0' 전까지 */
    s_ptr = s_ptr + 5;
    len = 0;
    while (s_ptr[len] != ',' && s_ptr[len] != '\0' && len < (MAX_SSID_LEN - 1)) {
        ssid_tmp[len] = s_ptr[len];
        len++;
    }
    ssid_tmp[len] = '\0';

    if (len <= 0)
        return;

    /* 기존 SSID 있으면 업데이트 */
    for (i = 0; i < MAX_NETWORKS; i++) {
        if (rssi_table[i].active && strcmp(rssi_table[i].ssid, ssid_tmp) == 0) {
            rssi_table[i].rssi = rssi_tmp;
            return;
        }
    }

    /* 빈 슬롯에 삽입 */
    for (i = 0; i < MAX_NETWORKS; i++) {
        if (!rssi_table[i].active) {
            strlcpy(rssi_table[i].ssid, ssid_tmp, MAX_SSID_LEN);
            rssi_table[i].rssi = rssi_tmp;
            rssi_table[i].active = true;
            return;
        }
    }

    /* 테이블이 꽉 찼으면(정책: 무시) */
    printk(KERN_INFO "RSSI_DRV: table full, drop: %s\n", ssid_tmp);
}

/* 한 줄(개행 제거된 상태)을 처리: BEGIN/END/SSID,RSSI */
static void handle_line(const char *line)
{
    if (!line || line[0] == '\0')
        return;

    if (strcmp(line, "BEGIN") == 0) {
        memset(rssi_table, 0, sizeof(rssi_table));
        return;
    }

    if (strcmp(line, "END") == 0) {
        /* 프레임 종료 마커: 특별 처리 없음 */
        return;
    }

    /* 일반 데이터 라인 */
    parse_and_update(line);
}

/* 유저가 /dev/... 에 write하면 테이블 갱신됨 */
static ssize_t dev_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos)
{
    char *k_buf;
    size_t copy_len;
    size_t i;

    (void)file;
    (void)ppos;

    if (len == 0)
        return 0;

    copy_len = min(len, (size_t)WRITE_MAX);

    k_buf = kmalloc(copy_len + 1, GFP_KERNEL);
    if (!k_buf)
        return -ENOMEM;

    if (copy_from_user(k_buf, buf, copy_len)) {
        kfree(k_buf);
        return -EFAULT;
    }
    k_buf[copy_len] = '\0';

    mutex_lock(&table_lock);

    /* 입력을 문자 단위로 받아서 line_buffer에 누적, 개행에서 한 줄 처리 */
    for (i = 0; i < copy_len; i++) {
        char c = k_buf[i];

        if (c == '\n' || c == '\r') {
            line_buffer[line_ptr] = '\0';
            if (line_ptr > 0)
                handle_line(line_buffer);
            line_ptr = 0;
        } else {
            if (line_ptr < (int)(sizeof(line_buffer) - 1)) {
                line_buffer[line_ptr++] = c;
            } else {
                /* 라인이 너무 길면 버리고 리셋 */
                line_ptr = 0;
            }
        }
    }

    mutex_unlock(&table_lock);
    kfree(k_buf);

    return (ssize_t)len; /* 관례상 원래 len 반환 */
}

/*
 * read는 "BEGIN\n...END\n" 형태로 출력
 * - ppos를 지원해서 cat/echo 등에서 여러 번 read해도 정상 동작
 */
static ssize_t dev_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
    char *response;
    int pos;
    int i;
    size_t copy_len;
    size_t remain;

    (void)file;

    if (len == 0)
        return 0;

    response = kmalloc(RESP_BUF_SIZE, GFP_KERNEL);
    if (!response)
        return -ENOMEM;

    mutex_lock(&table_lock);

    pos = 0;
    pos += scnprintf(response + pos, RESP_BUF_SIZE - pos, "BEGIN\n");

    for (i = 0; i < MAX_NETWORKS; i++) {
        if (rssi_table[i].active) {
            pos += scnprintf(response + pos, RESP_BUF_SIZE - pos,
                             "SSID:%s,RSSI:%d\n", rssi_table[i].ssid, rssi_table[i].rssi);
            if (pos >= (RESP_BUF_SIZE - 32))  /* 여유 없으면 중단 */
                break;
        }
    }

    pos += scnprintf(response + pos, RESP_BUF_SIZE - pos, "END\n");

    mutex_unlock(&table_lock);

    /* offset 처리 */
    if (*ppos >= pos) {
        kfree(response);
        return 0;
    }

    remain = (size_t)pos - (size_t)(*ppos);
    copy_len = min(len, remain);

    if (copy_to_user(buf, response + *ppos, copy_len)) {
        kfree(response);
        return -EFAULT;
    }

    *ppos += (loff_t)copy_len;
    kfree(response);

    return (ssize_t)copy_len;
}

static const struct file_operations fops = {
    .owner = THIS_MODULE,
    .read  = dev_read,
    .write = dev_write,
};

static int __init rssi_init(void)
{
    major_num = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_num < 0)
        return major_num;

    rssi_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(rssi_class)) {
        unregister_chrdev(major_num, DEVICE_NAME);
        return PTR_ERR(rssi_class);
    }

    rssi_device = device_create(rssi_class, NULL, MKDEV(major_num, 0), NULL, DEVICE_NAME);
    if (IS_ERR(rssi_device)) {
        class_destroy(rssi_class);
        unregister_chrdev(major_num, DEVICE_NAME);
        return PTR_ERR(rssi_device);
    }

    mutex_init(&table_lock);
    memset(rssi_table, 0, sizeof(rssi_table));
    memset(line_buffer, 0, sizeof(line_buffer));
    line_ptr = 0;

    printk(KERN_INFO "RSSI_DRV: Loaded. /dev/%s\n", DEVICE_NAME);
    return 0;
}

static void __exit rssi_exit(void)
{
    device_destroy(rssi_class, MKDEV(major_num, 0));
    class_unregister(rssi_class);
    class_destroy(rssi_class);
    unregister_chrdev(major_num, DEVICE_NAME);
    printk(KERN_INFO "RSSI_DRV: Unloaded\n");
}

module_init(rssi_init);
module_exit(rssi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ros");
MODULE_DESCRIPTION("RSSI table driver (BEGIN/END frame format)");
