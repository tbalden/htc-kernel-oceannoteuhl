/* Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "mdss_htc_util.h"
#include "mdss_dsi.h"
#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_panel.h"
#include <linux/CwMcuSensor.h>

struct attribute_status htc_attr_status[] = {
	{"cabc_level_ctl", 0, 0, 0},
	{"color_temp_ctl", 0, 0, 0},
	{"color_profile_ctl", 0, 0, 0},
	{"vddio_switch", 0, 0, 0},
	{"burst_switch", 0, 0, 0},
	{"bklt_cali_enable", 0, 0, 0},
	{"disp_cali_enable", 0, 0, 0},
};

#define DEBUG_BUF   2048
#define MIN_COUNT   9
#define DCS_MAX_CNT   128

static char debug_buf[DEBUG_BUF];
struct mdss_dsi_ctrl_pdata *ctrl_instance = NULL;
static char dcs_cmds[DCS_MAX_CNT];
static char *tmp;
static struct dsi_cmd_desc debug_cmd = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, 1}, dcs_cmds
};
static char dsi_rbuf[4];
static void dsi_read_cb(int len)
{
	unsigned *lp;

	lp = (uint32_t *)dsi_rbuf;
	pr_info("%s: data=0x%x len=%d\n", __func__,*lp, len);
}
static ssize_t dsi_cmd_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	u32 type = 0, value = 0;
	int cnt, i;
	struct dcs_cmd_req cmdreq;

	if (count >= sizeof(debug_buf) || count < MIN_COUNT)
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	if (!ctrl_instance)
		return count;

	
	debug_buf[count] = 0;

	
	cnt = (count) / 3 - 1;
	debug_cmd.dchdr.dlen = cnt;

	
	sscanf(debug_buf, "%x", &type);

	if (type == DTYPE_DCS_LWRITE)
		debug_cmd.dchdr.dtype = DTYPE_DCS_LWRITE;
	else if (type == DTYPE_GEN_LWRITE)
		debug_cmd.dchdr.dtype = DTYPE_GEN_LWRITE;
	else if (type == DTYPE_DCS_READ)
		debug_cmd.dchdr.dtype = DTYPE_DCS_READ;
	else
		return -EFAULT;

	pr_info("%s: cnt=%d, type=0x%x\n", __func__, cnt, type);

	
	for (i = 0; i < cnt; i++) {
		if (i >= DCS_MAX_CNT) {
			pr_info("%s: DCS command count over DCS_MAX_CNT, Skip these commands.\n", __func__);
			break;
		}
		tmp = debug_buf + (3 * (i + 1));
		sscanf(tmp, "%x", &value);
		dcs_cmds[i] = value;
		pr_info("%s: value=0x%x\n", __func__, dcs_cmds[i]);
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	memset(&dsi_rbuf, 0, sizeof(dsi_rbuf));

	if (type == DTYPE_DCS_READ){
		cmdreq.cmds = &debug_cmd;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_COMMIT | CMD_REQ_RX;
		cmdreq.rlen = 4;
		cmdreq.rbuf = dsi_rbuf;
		cmdreq.cb = dsi_read_cb; 

		mdss_dsi_cmdlist_put(ctrl_instance, &cmdreq);
	} else {
		cmdreq.cmds = &debug_cmd;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;
		mdss_dsi_cmdlist_put(ctrl_instance, &cmdreq);
		pr_info("%s %ld\n", __func__, count);
	}
	return count;
}

static const struct file_operations dsi_cmd_fops = {
	.write = dsi_cmd_write,
};

void htc_debugfs_init(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct dentry *dent = debugfs_create_dir("htc_debug", NULL);

	pr_info("%s\n", __func__);

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_instance = container_of(pdata, struct mdss_dsi_ctrl_pdata,
						panel_data);

	if (IS_ERR(dent)) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return;
	}

	if (debugfs_create_file("dsi_cmd", 0644, dent, 0, &dsi_cmd_fops)
			== NULL) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return;
	}
	return;
}

static struct calibration_gain aux_gain;
#define RGB_MIN_COUNT   9
static ssize_t rgb_gain_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%s%x\n%s%x\n%s%x\n", "GAIN_R=0x", aux_gain.R, "GAIN_G=0x",
				aux_gain.G, "GAIN_B=0x", aux_gain.B);
	return ret;
}

static ssize_t rgb_gain_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	unsigned temp, temp1, temp2;

	if (count < RGB_MIN_COUNT)
		return -EFAULT;

	

	if (sscanf(buf, "%x %x %x ", &temp, &temp1, &temp2) != 3) {
		pr_err("%s sscanf buf fail\n",__func__);
	} else if (RGB_GAIN_CHECK(temp) && RGB_GAIN_CHECK(temp1) && RGB_GAIN_CHECK(temp2)) {
		aux_gain.R = temp;
		aux_gain.G = temp1;
		aux_gain.B = temp2;
		pr_info("%s %d, gain_r=%x, gain_g=%x, gain_b=%x \n",__func__, __LINE__,
				aux_gain.R, aux_gain.G, aux_gain.B);
	}

	return count;
}

static ssize_t bklt_gain_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%s%d\n", "GAIN_BKLT=", aux_gain.BKL);
	return ret;
}

static ssize_t bklt_gain_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	int temp = 0;

	if (sscanf(buf, "%d", &temp) != 1) {
		pr_err("%s sscanf buf fail\n",__func__);
	} else if(BRI_GAIN_CHECK(temp)) {
		aux_gain.BKL = temp;
		pr_info("[DISP]%s %d, gain_bkl=%d \n",__func__, __LINE__, aux_gain.BKL);
	}

	return count;
}

static unsigned backlightvalue = 0;
static ssize_t camera_bl_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%s%u\n", "BL_CAM_MIN=", backlightvalue);
	return ret;
}

static ssize_t attrs_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		if (!strcmp(attr->attr.name, htc_attr_status[i].title)) {
			ret = scnprintf(buf, PAGE_SIZE, "%d\n", htc_attr_status[i].cur_value);
			break;
		}
	}

	return ret;
}

static ssize_t attr_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	unsigned long res;
	int rc, i;

	rc = kstrtoul(buf, 10, &res);
	if (rc) {
		pr_err("invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		if (!strcmp(attr->attr.name, htc_attr_status[i].title)) {
			htc_attr_status[i].req_value = res;
			break;
		}
	}

err_out:
	return count;
}

static ssize_t switch_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	int ret;
	struct msm_fb_data_type *mfd = dev_get_drvdata(dev->parent);
	ret = attr_store(dev, attr, buf, count);
	htc_set_vddio_switch(mfd);
	return ret;
}


#define SLEEPMS_OFFSET(strlen) (strlen+1) 
#define CMDLEN_OFFSET(strlen)  (SLEEPMS_OFFSET(strlen)+sizeof(const __be32))
#define CMD_OFFSET(strlen)     (CMDLEN_OFFSET(strlen)+sizeof(const __be32))

static struct __dsi_cmd_map{
	char *cmdtype_str;
	int  cmdtype_strlen;
	int  dtype;
} dsi_cmd_map[] = {
	{ "DTYPE_DCS_WRITE", 0, DTYPE_DCS_WRITE },
	{ "DTYPE_DCS_WRITE1", 0, DTYPE_DCS_WRITE1 },
	{ "DTYPE_DCS_LWRITE", 0, DTYPE_DCS_LWRITE },
	{ "DTYPE_GEN_WRITE", 0, DTYPE_GEN_WRITE },
	{ "DTYPE_GEN_WRITE1", 0, DTYPE_GEN_WRITE1 },
	{ "DTYPE_GEN_WRITE2", 0, DTYPE_GEN_WRITE2 },
	{ "DTYPE_GEN_LWRITE", 0, DTYPE_GEN_LWRITE },
	{ NULL, 0, 0 }
};

int htc_mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len = 0;
	char *buf;
	struct property *prop;
	struct dsi_ctrl_hdr *pdchdr;
	int i, cnt;
	int curcmdtype;

	i = 0;
	while (dsi_cmd_map[i].cmdtype_str) {
		if (!dsi_cmd_map[i].cmdtype_strlen) {
			dsi_cmd_map[i].cmdtype_strlen = strlen(dsi_cmd_map[i].cmdtype_str);
		}
		i++;
	}

	prop = of_find_property( np, cmd_key, &len);
	if (!prop || !len || !(prop->length) || !(prop->value)) {
		pr_err("%s: failed, key=%s  [%d : %d : %p]\n", __func__, cmd_key,
			len, (prop ? prop->length : -1), (prop ? prop->value : 0) );
		
		return -ENOMEM;
	}

	data = prop->value;
	blen = 0;
	cnt = 0;
	while (len > 0) {
		curcmdtype = 0;
		while (dsi_cmd_map[curcmdtype].cmdtype_strlen) {
			if( !strncmp( data, dsi_cmd_map[curcmdtype].cmdtype_str,
						dsi_cmd_map[curcmdtype].cmdtype_strlen ) &&
				data[dsi_cmd_map[curcmdtype].cmdtype_strlen] == '\0' )
				break;
			curcmdtype++;
		};
		if( !dsi_cmd_map[curcmdtype].cmdtype_strlen ) 
			break;

		i = be32_to_cpup((__be32 *)&data[CMDLEN_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)]);
		blen += i;
		cnt++;

		data = data + CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen) + i;
		len = len - CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen) - i;
	}

	if(len || !cnt || !blen){
		pr_err("%s: failed, key[%s] : %d cmds, remain=%d bytes \n", __func__, cmd_key, cnt, len);
		return -ENOMEM;
	}

	i = (sizeof(char)*blen+sizeof(struct dsi_ctrl_hdr)*cnt);
	buf = kzalloc( i, GFP_KERNEL);
	if (!buf){
		pr_err("%s: create dsi ctrl oom failed \n", __func__);
		return -ENOMEM;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc), GFP_KERNEL);
	if (!pcmds->cmds){
		pr_err("%s: create dsi commands oom failed \n", __func__);
		goto exit_free;
	}

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = i;
	data = prop->value;
	for(i=0; i<cnt; i++){
		pdchdr = &pcmds->cmds[i].dchdr;

		curcmdtype = 0;
		while(dsi_cmd_map[curcmdtype].cmdtype_strlen){
			if( !strncmp( data, dsi_cmd_map[curcmdtype].cmdtype_str,
						dsi_cmd_map[curcmdtype].cmdtype_strlen ) &&
				data[dsi_cmd_map[curcmdtype].cmdtype_strlen] == '\0' ){
				pdchdr->dtype = dsi_cmd_map[curcmdtype].dtype;
				break;
			}
			curcmdtype ++;
		}

		pdchdr->last = 0x01;
		pdchdr->vc = 0x00;
		pdchdr->ack = 0x00;
		pdchdr->wait = be32_to_cpup((__be32 *)&data[SLEEPMS_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)]) & 0xff;
		pdchdr->dlen = be32_to_cpup((__be32 *)&data[CMDLEN_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)]);
		memcpy( buf, pdchdr, sizeof(struct dsi_ctrl_hdr) );
		buf += sizeof(struct dsi_ctrl_hdr);
		memcpy( buf, &data[CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)], pdchdr->dlen);
		pcmds->cmds[i].payload = buf;
		buf += pdchdr->dlen;
		data = data + CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen) + pdchdr->dlen;
	}

	data = of_get_property(np, link_key, NULL);
	if (data) {
		if (!strncmp(data, "dsi_hs_mode", 11))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	} else {
		pcmds->link_state = DSI_HS_MODE;
	}
	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}

static DEVICE_ATTR(backlight_info, S_IRUGO, camera_bl_show, NULL);
static DEVICE_ATTR(cabc_level_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(color_temp_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(color_profile_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(vddio_switch, S_IRUGO | S_IWUSR, attrs_show, switch_store);
static DEVICE_ATTR(burst_switch, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(bklt_cali, S_IRUGO | S_IWUSR, bklt_gain_show, bklt_gain_store);
static DEVICE_ATTR(bklt_cali_enable, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(disp_cali, S_IRUGO | S_IWUSR, rgb_gain_show, rgb_gain_store);
static DEVICE_ATTR(disp_cali_enable, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static struct attribute *htc_extend_attrs[] = {
	&dev_attr_backlight_info.attr,
	&dev_attr_cabc_level_ctl.attr,
	&dev_attr_color_temp_ctl.attr,
	&dev_attr_color_profile_ctl.attr,
	&dev_attr_vddio_switch.attr,
	&dev_attr_burst_switch.attr,
	&dev_attr_bklt_cali.attr,
	&dev_attr_bklt_cali_enable.attr,
	&dev_attr_disp_cali.attr,
	&dev_attr_disp_cali_enable.attr,
	NULL,
};

static struct attribute_group htc_extend_attr_group = {
	.attrs = htc_extend_attrs,
};

void htc_register_attrs(struct kobject *led_kobj, struct msm_fb_data_type *mfd)
{
	int rc;
	struct mdss_panel_data *pdata = dev_get_platdata(&mfd->pdev->dev);
	struct calibration_gain *gain = &(pdata->panel_info.cali_gain);

	pr_err("htc_register_attrs\n");

	rc = sysfs_create_group(led_kobj, &htc_extend_attr_group);
	if (rc)
		pr_err("sysfs group creation failed, rc=%d\n", rc);

	mfd->compass_notifier_block.notifier_call = compass_notifier_fn;
	compass_en_register_notifier(&mfd->compass_notifier_block);

	
	if (RGB_GAIN_CHECK(gain->R) && RGB_GAIN_CHECK(gain->G) && RGB_GAIN_CHECK(gain->B)) {
		aux_gain.R = gain->R;
		aux_gain.G = gain->G;
		aux_gain.B = gain->B;
	}

	
	if (BRI_GAIN_CHECK(gain->BKL))
		aux_gain.BKL = gain->BKL;

	return;
}

void htc_reset_status(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		htc_attr_status[i].cur_value = htc_attr_status[i].def_value;
	}

	return;
}

static ssize_t htc_set_bl_sync(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(dev->parent);

	if (strtobool(buf, &mfd->bl_sync) < 0)
		return -EINVAL;

	return count;
}

static ssize_t htc_get_bl_sync(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(dev->parent);

	return scnprintf(buf, PAGE_SIZE, "%d\n", mfd->bl_sync);
}

static DEVICE_ATTR(bl_sync, S_IRUGO | S_IWUSR, htc_get_bl_sync, htc_set_bl_sync);

static struct attribute *htc_sub_attrs[] = {
	&dev_attr_bl_sync.attr,
	NULL
};

static struct attribute_group htc_sub_attr_group = {
	.attrs = htc_sub_attrs,
};

void htc_register_sub_attrs(struct kobject *led_kobj)
{
	int rc;
	rc = sysfs_create_group(led_kobj, &htc_sub_attr_group);
	if (rc)
		pr_err("%s: sysfs group creation failed, rc=%d\n", __func__, rc);

	return;
}

void htc_unregister_sub_attrs(struct kobject *led_kobj)
{
	sysfs_remove_group(led_kobj, &htc_sub_attr_group);
}

void htc_register_camera_bkl(int level)
{
	backlightvalue = level;
}

void htc_set_cabc(struct msm_fb_data_type *mfd, bool force)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (htc_attr_status[CABC_INDEX].req_value > 2)
		return;

	if (!ctrl_pdata->cabc_off_cmds.cmds)
		return;

	if (!ctrl_pdata->cabc_ui_cmds.cmds)
		return;

	if (!ctrl_pdata->cabc_video_cmds.cmds)
		return;

	if (!force && (htc_attr_status[CABC_INDEX].req_value == htc_attr_status[CABC_INDEX].cur_value))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (htc_attr_status[CABC_INDEX].req_value == 0) {
		cmdreq.cmds = ctrl_pdata->cabc_off_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_off_cmds.cmd_cnt;
	} else if (htc_attr_status[CABC_INDEX].req_value == 1) {
		cmdreq.cmds = ctrl_pdata->cabc_ui_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_ui_cmds.cmd_cnt;
	} else if (htc_attr_status[CABC_INDEX].req_value == 2) {
		cmdreq.cmds = ctrl_pdata->cabc_video_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_video_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->cabc_ui_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_ui_cmds.cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[CABC_INDEX].cur_value = htc_attr_status[CABC_INDEX].req_value;
	pr_info("%s: cabc mode=%d\n", __func__, htc_attr_status[CABC_INDEX].cur_value);
	return;
}

void htc_set_color_temp(struct msm_fb_data_type *mfd, bool force)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;
	int req_mode = 0;
	int i = 0;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (!ctrl_pdata->color_temp_cnt)
		return;

	for (i = 0; i < ctrl_pdata->color_temp_cnt ; i++) {
		if (!ctrl_pdata->color_temp_cmds[i].cmds)
			return;
	}

	if (htc_attr_status[COLOR_TEMP_INDEX].req_value >= ctrl_pdata->color_temp_cnt)
		return;

	if (!force && (htc_attr_status[COLOR_TEMP_INDEX].req_value == htc_attr_status[COLOR_TEMP_INDEX].cur_value))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	req_mode = htc_attr_status[COLOR_TEMP_INDEX].req_value;
	cmdreq.cmds = ctrl_pdata->color_temp_cmds[req_mode].cmds;
	cmdreq.cmds_cnt = ctrl_pdata->color_temp_cmds[req_mode].cmd_cnt;

	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[COLOR_TEMP_INDEX].cur_value = htc_attr_status[COLOR_TEMP_INDEX].req_value;
	pr_info("%s: color temp mode=%d\n", __func__, htc_attr_status[COLOR_TEMP_INDEX].cur_value);
	return;
}

void htc_set_color_profile(struct msm_fb_data_type *mfd, bool force)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (!ctrl_pdata->color_default_cmds.cmd_cnt || !ctrl_pdata->color_srgb_cmds.cmd_cnt)
		return;

	if ((htc_attr_status[COLOR_PROFILE_INDEX].req_value > SRGB_MODE) || (htc_attr_status[COLOR_PROFILE_INDEX].req_value < DEFAULT_MODE))
		return;

	if (!force && (htc_attr_status[COLOR_PROFILE_INDEX].req_value == htc_attr_status[COLOR_PROFILE_INDEX].cur_value))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (htc_attr_status[COLOR_PROFILE_INDEX].req_value == SRGB_MODE) {
		cmdreq.cmds = ctrl_pdata->color_srgb_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->color_srgb_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->color_default_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->color_default_cmds.cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[COLOR_PROFILE_INDEX].cur_value = htc_attr_status[COLOR_PROFILE_INDEX].req_value;
	pr_info("%s: color profile mode=%d\n", __func__, htc_attr_status[COLOR_PROFILE_INDEX].cur_value);
	return;
}

void compass_set_vddio_switch(struct msm_fb_data_type *mfd, int enable)
{
	struct mdss_mdp_ctl *ctl = mfd_to_ctl(mfd);
	int event = enable ? MDSS_EVENT_PANEL_VDDIO_SWITCH_ON : MDSS_EVENT_PANEL_VDDIO_SWITCH_OFF;

	mdss_mdp_ctl_intf_event(ctl, event, NULL, CTL_INTF_EVENT_FLAG_DEFAULT);
}

void htc_set_vddio_switch(struct msm_fb_data_type *mfd)
{
	if (htc_attr_status[VDDIO_INDEX].req_value == htc_attr_status[VDDIO_INDEX].cur_value)
		return;

	compass_set_vddio_switch(mfd, htc_attr_status[VDDIO_INDEX].req_value);

	htc_attr_status[VDDIO_INDEX].cur_value = htc_attr_status[VDDIO_INDEX].req_value;
	pr_info("%s: vddio switch=%d\n", __func__, htc_attr_status[VDDIO_INDEX].cur_value);
	return;
}

int compass_notifier_fn(struct notifier_block *nb,
                        unsigned long action, void *data)
{
	struct msm_fb_data_type *mfd;
	mfd = container_of(nb, struct msm_fb_data_type, compass_notifier_block);
	pr_info("%s: action=%d\n", __func__, (int)action);

	compass_set_vddio_switch(mfd, (action) ? 1 : 0);

	return 0;
}

void htc_set_burst(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;
	static int burst_mode = 0;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (ctrl_pdata->burst_on_level == 0 || ctrl_pdata->burst_off_level == 0)
		return;

	if (!ctrl_pdata->burst_off_cmds.cmds)
		return;

	if (!ctrl_pdata->burst_on_cmds.cmds)
		return;

	if (htc_attr_status[BURST_SWITCH_INDEX].req_value == htc_attr_status[BURST_SWITCH_INDEX].cur_value)
		return;

	if (burst_mode == 1 && htc_attr_status[BURST_SWITCH_INDEX].req_value <= ctrl_pdata->burst_off_level) {
		burst_mode = 0;
	} else if (burst_mode == 0 && htc_attr_status[BURST_SWITCH_INDEX].req_value >= ctrl_pdata->burst_on_level){
		burst_mode = 1;
	} else {
		htc_attr_status[BURST_SWITCH_INDEX].cur_value = htc_attr_status[BURST_SWITCH_INDEX].req_value;
		return;
	}

	if(!mdss_fb_is_power_on(mfd))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (burst_mode) {
		cmdreq.cmds = ctrl_pdata->burst_on_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->burst_on_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->burst_off_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->burst_off_cmds.cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[BURST_SWITCH_INDEX].cur_value = htc_attr_status[BURST_SWITCH_INDEX].req_value;
	pr_info("%s burst mode=%d\n", __func__, htc_attr_status[BURST_SWITCH_INDEX].cur_value);
	return;
}

bool htc_is_burst_bl_on(struct msm_fb_data_type *mfd, int value)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	struct htc_backlight1_table *brt_bl_table = &panel_info->brt_bl_table[0];
	int size = brt_bl_table->size;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	
	if (mfd->panel_info->burst_bl_value == 0)
		return false;

	if(!size || size < 2 || !brt_bl_table->brt_data)
		return false;

	if (ctrl_pdata->burst_on_level == 0 || ctrl_pdata->burst_off_level == 0)
		return false;

	if (htc_attr_status[BURST_SWITCH_INDEX].req_value < 0)
		return false;

	htc_attr_status[BURST_SWITCH_INDEX].cur_value = htc_attr_status[BURST_SWITCH_INDEX].req_value;

	pr_debug("%s burst level=%d, value=%d, max brt=%d\n", __func__,
		htc_attr_status[BURST_SWITCH_INDEX].cur_value, value, brt_bl_table->brt_data[size - 1]);

	if (htc_attr_status[BURST_SWITCH_INDEX].cur_value >= ctrl_pdata->burst_on_level) {
		if(value >= brt_bl_table->brt_data[size - 1])
			return true;
	}

	return false;
}

static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds, u32 flags)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = flags;

	
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;
	else if (pcmds->link_state == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}



static ssize_t htc_fb_set_aod_ctrl(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	unsigned long res;
	int rc = 0;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	rc = kstrtoul(buf, 10, &res);
	if (rc) {
		pr_err("%s: invalid parameter, %s, rc=%d\n", __func__, buf, rc);
		count = -EINVAL;
		goto err_out;
	}
	if (res >= FB_AOD_MAX) {
		pr_err("%s: invalid parameter for req_state=%lu\n", __func__, res);
		count = -EINVAL;
		goto err_out;
	}

	mutex_lock(&mfd->aod_lock);
	if (res == panel_info->aod.req_state)
		goto unlock;

	if (!mdss_fb_is_power_on(mfd)) {
		pr_info("%s: Request AOD state from %d to %lu during screen off\n", __func__, panel_info->aod.req_state, res);
	} else if (panel_info->aod.power_state == FB_AOD_FULL_ON && res == FB_AOD_PARTIAL_ON) {
		panel_info->aod.next_state = FB_AOD_PARTIAL_ON;
		pr_info("%s: Change to AOD Partial Mode\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->aod_cmds[0], CMD_REQ_COMMIT);
	} else if (panel_info->aod.power_state == FB_AOD_PARTIAL_ON && res == FB_AOD_FULL_ON) {
		panel_info->aod.next_state = FB_AOD_FULL_ON;
		pr_info("%s: Change to AOD Full Mode\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->aod_cmds[1], CMD_REQ_COMMIT);
	} else {
		
		pr_info("%s: Request AOD state from %d to %lu\n", __func__, panel_info->aod.req_state, res);
	}

	panel_info->aod.req_state = res;
	aod_send_notify(fbi);

unlock:
	mutex_unlock(&mfd->aod_lock);

err_out:
	return count;
}

static ssize_t htc_fb_get_aod_ctrl(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	ret = scnprintf(buf, PAGE_SIZE, "req_state=%d, power_state=%d\n",
		panel_info->aod.req_state, panel_info->aod.power_state);

	return ret;
}

static ssize_t htc_fb_set_sw49407_debug(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	unsigned long res;
	int rc = 0;

	rc = kstrtoul(buf, 10, &res);
	if (rc)
		return -EINVAL;

	panel_info->aod.debug = res;

	return count;
}

static ssize_t htc_fb_get_sw49407_debug(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	return scnprintf(buf, PAGE_SIZE, "%d\n", panel_info->aod.debug);
}

static DEVICE_ATTR(aod_ctrl, S_IRUGO | S_IWUSR, htc_fb_get_aod_ctrl, htc_fb_set_aod_ctrl);
static DEVICE_ATTR(sw49407_debug, S_IRUGO | S_IWUSR, htc_fb_get_sw49407_debug, htc_fb_set_sw49407_debug);

static struct attribute *mdss_fb_aod_attrs[] = {
	&dev_attr_aod_ctrl.attr,
	&dev_attr_sw49407_debug.attr,
	NULL
};

static struct attribute_group mdss_fb_aod_attr_group = {
	.attrs = mdss_fb_aod_attrs,
};

int htc_mdss_fb_create_sysfs(struct msm_fb_data_type *mfd)
{
	int rc = 0;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	if (panel_info->aod.supported) {
		rc = sysfs_create_group(&mfd->fbi->dev->kobj, &mdss_fb_aod_attr_group);
		if (rc)
			pr_err("sysfs file creation failed, rc=%d\n", rc);
	}

	return rc;
}

void htc_mdss_fb_remove_sysfs(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_info *panel_info = mfd->panel_info;

	if (panel_info->aod.supported)
		sysfs_remove_group(&mfd->fbi->dev->kobj, &mdss_fb_aod_attr_group);
}

int aod_send_notify(struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct aod_panel_info *aod = NULL;

	if (!mfd || !mfd->panel_info)
		return -EINVAL;

	aod = &mfd->panel_info->aod;

	if (aod->supported) {
		struct fb_event event;
		int value;

		if (aod->power_state != aod->next_state) {
			value = aod->next_state;
			event.info = info;
			event.data = &value;

			pr_info("[DISP] Send AOD_MODE notify %d\n", value);
			fb_notifier_call_chain(FB_EVENT_AOD_MODE, &event);
		}
		aod->power_state = aod->next_state;

		return 1;
	}

	return 0;
}

void htc_update_bl_cali_data(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct calibration_gain *gain = NULL;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	struct htc_backlight1_table *brt_bl_table = &panel_info->brt_bl_table[0];
	int size = brt_bl_table->size;
	int bl_lvl = 0;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	gain = &(pdata->panel_info.cali_gain);

	if (pdata->panel_info.cali_data_format == PANEL_CALIB_NOT_SUPPORTED)
		return;

	if (htc_attr_status[BL_CALI_ENABLE_INDEX].cur_value == htc_attr_status[BL_CALI_ENABLE_INDEX].req_value)
		return;

	htc_attr_status[BL_CALI_ENABLE_INDEX].cur_value = htc_attr_status[BL_CALI_ENABLE_INDEX].req_value;

	
	if ((aux_gain.BKL != gain->BKL)) {
		gain->BKL = aux_gain.BKL;
		pr_info("%s change bkl calibration value, bkl=%d\n", __func__, gain->BKL);
	}

	if (!BRI_GAIN_CHECK(gain->BKL)) {
		pr_info("%s bkl=%d out of range\n", __func__, gain->BKL);
		return;
	}

	brt_bl_table->apply_cali = htc_attr_status[BL_CALI_ENABLE_INDEX].cur_value;

	
	kfree(brt_bl_table->bl_data);
	brt_bl_table->bl_data = kzalloc(size * sizeof(u16), GFP_KERNEL);
	if (!brt_bl_table->bl_data) {
		pr_err("unable to allocate memory for bl_data\n");
		return;
	}
	memcpy(brt_bl_table->bl_data, brt_bl_table->bl_data_raw, size * sizeof(u16));

	
	if (brt_bl_table->apply_cali) {
		u16 *bl_data_raw;
		u16 tmp_cali_value = 0;

		
		if(!size || size < 2 || !brt_bl_table->brt_data || !brt_bl_table->bl_data)
			return;

		if (pdata->panel_info.cali_data_format == PANEL_CALIB_REV_1) {
			bl_data_raw = brt_bl_table->bl_data_raw;

			
			tmp_cali_value = BACKLIGHT_CALI(bl_data_raw[0], gain->BKL);
			brt_bl_table->bl_data[0] = VALID_CALI_BKLT(tmp_cali_value, 0, bl_data_raw[1]);
			tmp_cali_value = BACKLIGHT_CALI(bl_data_raw[size - 1], gain->BKL);
			brt_bl_table->bl_data[size - 1] = VALID_CALI_BKLT(tmp_cali_value, bl_data_raw[size - 2], panel_info->bl_max);
		}
	}

	if (mfd->bl_level && mfd->last_bri1) {
		
		bl_lvl = mdss_backlight_trans(mfd->last_bri1, &mfd->panel_info->brt_bl_table[0], true);

		
		if (bl_lvl) {
			mfd->allow_bl_update = false;
			mfd->unset_bl_level = bl_lvl;
		}
	}

	pr_info("%s bl_cali=%d, unset_bl_level=%d \n", __func__, gain->BKL,  mfd->unset_bl_level);
}
void htc_update_rgb_cali_data(struct msm_fb_data_type *mfd, struct mdp_pcc_cfg_data *config)
{
	struct mdss_panel_data *pdata;
	struct calibration_gain *gain = NULL;
	struct mdp_pcc_data_v1_7 *pcc_data = config->cfg_payload;


	pdata = dev_get_platdata(&mfd->pdev->dev);

	gain = &(pdata->panel_info.cali_gain);

	if (pdata->panel_info.cali_data_format == PANEL_CALIB_NOT_SUPPORTED)
		return;

	htc_attr_status[RGB_CALI_ENABLE_INDEX].cur_value = htc_attr_status[RGB_CALI_ENABLE_INDEX].req_value;

	
	if ((aux_gain.R != gain->R) || (aux_gain.G != gain->G) || (aux_gain.B != gain->B)) {
		gain->R = aux_gain.R;
		gain->G = aux_gain.G;
		gain->B = aux_gain.B;
		pr_info("%s change calibration value, RGB(0x%x, 0x%x, 0x%x) \n",
				__func__, gain->R, gain->G, gain->B);
	}

	if (!RGB_GAIN_CHECK(gain->R) || !RGB_GAIN_CHECK(gain->G) || !RGB_GAIN_CHECK(gain->B)) {
		pr_info("%s RGB(0x%x, 0x%x, 0x%x) out of range\n", __func__, gain->R, gain->G, gain->B);
		return;
	}

	
	if (htc_attr_status[RGB_CALI_ENABLE_INDEX].cur_value) {
		pcc_data->r.r = RGB_CALIBRATION(pcc_data->r.r, gain->R);
		pcc_data->g.g = RGB_CALIBRATION(pcc_data->g.g, gain->G);
		pcc_data->b.b = RGB_CALIBRATION(pcc_data->b.b, gain->B);

		pr_info("%s apply calibration value, RGB(0x%x, 0x%x, 0x%x) \n",
			__func__, gain->R, gain->G, gain->B);
	}
}
