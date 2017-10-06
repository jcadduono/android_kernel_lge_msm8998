
/*mausb_misc.c used to write to FTM item in MISC partition*/
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#define MISC_PATH "/dev/block/platform/soc/7824900.sdhci/by-name/misc"

#define LGFTM_ITEM_MAX          3585

unsigned int android_mausb = 0;

struct file *misc_open(void)
{
	struct file *filp;

	filp = filp_open(MISC_PATH, O_RDWR, S_IRUSR|S_IWUSR);

	if (IS_ERR(filp)) {
		pr_err("Can't open misc partition : %d error:%ld \n", IS_ERR(filp),PTR_ERR(filp));
		return NULL;
	}

	return filp;
}

static int check_index(int id)
{
	if (id < 0 || id > LGFTM_ITEM_MAX)
		return -1;

	return 0;
}

int set_ftm_item(int id, int size, void *in)
{
	struct file *filp;
	int ret = 0;
	int len;
	void *data;
	mm_segment_t oldfs;

	if (check_index(id)) {
		pr_err("Invalid id of ftm_item, %d\n", id);
		return -1;
	}

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	filp = misc_open();

	if (filp != NULL) {
		filp->f_pos = id * 1024 * 2;

		data = kmalloc(size, GFP_KERNEL);
		if (data == NULL) {
			pr_err("Failed to allocate buffer\n");
			ret = -1;
			goto fail_mem_allocate;
		}

		memcpy(data, in, size);

		len = vfs_write(filp, (char *)data, size, &filp->f_pos);
		if (len < 0) {
			pr_err("Failed to read ftm_item\n");
			ret = -1;
			goto fail_vfs_read;
		}
	} else {
		ret = -1;
		goto fail_misc_open;
	}

fail_vfs_read:
	kfree(data);
fail_mem_allocate:
	filp_close(filp, NULL);
fail_misc_open:
	set_fs(oldfs);
	return ret;
}



int __init lge_android_mausb(char *s)
{
	if(!strcmp(s,"enable"))
		android_mausb = 1;
	else
		android_mausb = 0;
	printk("mausb_status = %d\n", android_mausb);

	return 1;
}
__setup("mausb_status=", lge_android_mausb);

unsigned int lge_get_android_mausb(void)
{
	return android_mausb;
}
