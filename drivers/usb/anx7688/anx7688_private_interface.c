/******************************************************************************

Copyright (c) 2016, Analogix Semiconductor, Inc.

PKG Ver  : V2.1.11

Filename : anx7688_private_interface.c

Project  : ANX7688

Created  : 28 Nov. 2016

Devices  : ANX7688

Toolchain: Android

Description:

Revision History:

******************************************************************************/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/rwlock_types.h>
#include <linux/completion.h>
#include <linux/power/htc_battery.h>
#include "anx7688_private_interface.h"
#include "anx7688_public_interface.h"

extern unsigned char device_addr;

static u32 init_src_caps[1] = {
    
    
    
    PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_900MA, PDO_FIXED_FLAGS)
};

static u32 init_snk_cap[2] = {
    
    PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_900MA, PDO_FIXED_FLAGS),
    
    PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_3A, PDO_FIXED_FLAGS)
    
    
    
    
};
static u8 init_svid[4] = { 0x00, 0x00, 0x01, 0xff };
static u8 init_snk_ident[16] = { 0x00, 0x00, 0x00, 0xec,	
                                 0x00, 0x00, 0x00, 0x00,	
                                 0x00, 0x00, 0x00, 0x00,	
                                 0x39, 0x00, 0x00, 0x51		
                               };

u8 pd_src_pdo_cnt = 1;
u8 pd_src_pdo[VDO_SIZE] = {
    
    0x5A, 0x90, 0x01, 0x2A
};

u8 pd_snk_pdo_cnt = 2;
u8 pd_snk_pdo[VDO_SIZE];
u8 pd_rdo[PD_ONE_DATA_OBJECT_SIZE];
u8 DP_caps[PD_ONE_DATA_OBJECT_SIZE];
u8 configure_DP_caps[PD_ONE_DATA_OBJECT_SIZE];
u8 src_dp_caps[PD_ONE_DATA_OBJECT_SIZE];
unsigned char downstream_pd_cap = 0;

#define MAX_SEND_BUF_SIZE 8
#define MAX_RECV_BUF_SIZE 8

unsigned char pbuf_rx_front = 0;
unsigned char pbuf_tx_rear = 0;

#define TX_BUF_FRONT 0x11
#define TX_BUF_REAR   0x12
#define TX_BUF_START 0x18	

#define RX_BUF_FRONT 0x13
#define RX_BUF_REAR   0x14
#define RX_BUF_START  0x20	

#define RCVDER_ACK_STATUS 0x15
#define SENDER_ACK_STATUS 0x16


#define tx_buf_rear() pbuf_tx_rear
#define rx_buf_front() pbuf_rx_front

#define rx_buf_rear() ReadReg(RX_BUF_REAR)
#define tx_buf_front() ReadReg(TX_BUF_FRONT)

#define receiver_set_ack_status(val) WriteReg(RCVDER_ACK_STATUS, val)
#define receiver_get_ack_status() ReadReg(RCVDER_ACK_STATUS)

#define sender_get_ack_status() ((ReadReg(SENDER_ACK_STATUS)) & 0x7F)
#define sender_set_ack_status(val) WriteReg(SENDER_ACK_STATUS, val)

extern void dwc3_pd_vbus_ctrl(int on);
extern int usb_set_dwc_property(int prop_type,unsigned int value);
DECLARE_COMPLETION(pr_swap_rsp);
DECLARE_COMPLETION(dr_swap_rsp);


#define TRY_ROLE_TIMEOUT  600
#define WAIT_VBUS_TIME  1200
u8 try_source(void)
{
    unsigned long expire = 0;
    pr_info("Try source start.\n");
    if(!(ReadReg(0x40)&0x08)) {
        pr_info("Current role is DFP, no need Try source\n");
        return 1;
    }

    if(downstream_pd_cap) {
        return interface_pr_swap();
    } else {
        WriteReg(0x05,0x10);  	 
        WriteReg(0x4a,ReadReg(0x4a)|0x01); 
        
        WriteReg(0x6e,0x01);  	 

        WriteReg(0x40, ReadReg(0x40)|0x02); 
        expire = msecs_to_jiffies(TRY_ROLE_TIMEOUT) + jiffies;

        while(!(ReadReg(0x48)&0x0f))	 {
            if (time_before(expire, jiffies)) {
                pr_info("Try source timeout!0x48 = %x\n", ReadReg(0x48));
                goto try_src_fail;
            }
        }
#ifdef SUP_VBUS_CTL
        anx7688_vbus_control(1);
#endif
        WriteReg(0x05,0x0);  
        mdelay(50);
        WriteReg(0x6e,0x0);  	 
        if(ReadReg(0x40)&0x08) {
            pr_info("try source swap fail! \n");
            return 1;		      
        } else {
            pr_info("try source swap success! \n");
            return 0; 			
        }
    }
try_src_fail:
    pr_info("try source fail!\n");
    WriteReg(0x40, ReadReg(0x40)&0xfd);  
    WriteReg(0x05,0x00);  
    mdelay(1);
    WriteReg(0x69,0x19);  
    WriteReg(0x6e,0x0);  	 
    return 1;


}

u8 try_sink(void)
{
    unsigned long expire = 0;
    pr_info("Try sink start.\n");
    if(ReadReg(0x40)&0x08) {
        pr_info("Current role is UFP, no need Try sink\n");
        return 1;
    }

    if(downstream_pd_cap) {
        return interface_pr_swap();
    } else {
        WriteReg(0x05,0x10);  	 
        WriteReg(0x4a,ReadReg(0x4a) | 0x01); 
#ifdef SUP_VBUS_CTL
        anx7688_vbus_control(0);	
#endif
        WriteReg(0x3f,ReadReg(0x3f) | 0xdf); 
        WriteReg(0x36,ReadReg(0x36) | 0x80);
        

        WriteReg(0x40, ReadReg(0x40) & 0xfd);  

        expire = msecs_to_jiffies(TRY_ROLE_TIMEOUT) + jiffies;

        while(!(ReadReg(0x0d) & 0xfc))	 {
            if (time_before(expire, jiffies)) {
                pr_info("Try sink timeout!0x0d = %x\n", ReadReg(0x0d));
                goto try_sink_fail;
            }
        }

        mdelay(650);
        expire = msecs_to_jiffies(WAIT_VBUS_TIME) + jiffies;

        while(!(ReadReg(0x40) & 0x10)) {
            if (time_before(expire, jiffies)) {
                pr_info("wait vbus timeout!0x40 = %x\n", ReadReg(0x40));
                goto try_sink_fail;
            }
        }

        WriteReg(0x36,ReadReg(0x36) & 0x7f); 
        WriteReg(0x05,0x00);  
        if(ReadReg(0x40)&0x08) {
            pr_info("try sink swap success! \n");
            return 0;		      
        } else {
            pr_info("try sink  swap fail! \n");
            return 1; 			  
        }
    }
try_sink_fail:
    pr_info("try sink fail!\n");
    WriteReg(0x40, ReadReg(0x40)|0x02);  
    WriteReg(0x36,ReadReg(0x36) & 0x7f); 
    WriteReg(0x05,0x00);
    return 1;

}

u8 get_otp_indicator_byte(void)
{
    u8 temp;

    WriteReg(0xe5, 0xa0); 
    WriteReg(0xef, 0x7a); 
    WriteReg(0xd0, 0x00);   
    WriteReg(0xd1, 0x01); 		
    WriteReg(0xe5, 0xa1); 
    while(ReadReg(0xed) & 0x30); 
    temp = ReadReg(0xe0);
    WriteReg(0xef, 0x0); 
    WriteReg(0xe5, 0x0); 
    return temp;
}

s8 get_data_role(void)
{
    u8 status;

    
    status = ReadReg(SYSTEM_STSTUS);

    return ((status & DATA_ROLE) != 0);

}

s8 get_power_role(void)
{
    u8 status ;

    
    status = ReadReg(0x40);

    return ((status & 0x08) == 0);
}

u8 get_src_cap(const u8 *src_caps, u8 src_caps_size)
{

    return 1;
}

u8 get_snk_cap(u8 *snk_caps, u8 snk_caps_len)
{

    return 1;
}
u8 send_src_cap(const u8 *src_caps, u8 src_caps_size)
{
    if (NULL == src_caps)
        return CMD_FAIL;
    if ((src_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
        (src_caps_size / PD_ONE_DATA_OBJECT_SIZE) >
        PD_MAX_DATA_OBJECT_NUM) {
        return CMD_FAIL;
    }
    memcpy(pd_src_pdo, src_caps, src_caps_size);
    pd_src_pdo_cnt = src_caps_size / PD_ONE_DATA_OBJECT_SIZE;

    
    return interface_send_msg_timeout(TYPE_PWR_SRC_CAP, pd_src_pdo,
                                      pd_src_pdo_cnt *
                                      PD_ONE_DATA_OBJECT_SIZE,
                                      INTERFACE_TIMEOUT);
}

u8 send_snk_cap(const u8 *snk_caps, u8 snk_caps_size)
{
    memcpy(pd_snk_pdo, snk_caps, snk_caps_size);
    pd_snk_pdo_cnt = snk_caps_size / PD_ONE_DATA_OBJECT_SIZE;

    
    return interface_send_msg_timeout(TYPE_PWR_SNK_CAP, pd_snk_pdo,
                                      pd_snk_pdo_cnt * 4,
                                      INTERFACE_TIMEOUT);
}

u8 send_dp_snk_cfg(const u8 *dp_snk_caps, u8 dp_snk_caps_size)
{
    memcpy(configure_DP_caps, dp_snk_caps, dp_snk_caps_size);
    
    return interface_send_msg_timeout(TYPE_DP_SNK_CFG, configure_DP_caps,
                                      4, INTERFACE_TIMEOUT);
}

u8 send_src_dp_cap(const u8 *dp_caps, u8 dp_caps_size)
{
    if (NULL == dp_caps)
        return CMD_FAIL;
    if ((dp_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
        (dp_caps_size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
        return CMD_FAIL;
    }

    memcpy(src_dp_caps, dp_caps, dp_caps_size);

    
    return interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
                                      src_dp_caps, dp_caps_size,
                                      INTERFACE_TIMEOUT);
}


u8 send_dp_snk_identity(const u8 *snk_ident, u8 snk_ident_size)
{
    return interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
                                      (u8 *) snk_ident, snk_ident_size,
                                      INTERFACE_TIMEOUT);
}

u8 send_vdm(const u8 *vdm, u8 size)
{
    u8 tmp[32] = { 0 };
    if (NULL == vdm)
        return CMD_FAIL;
    if (size > 3 && size < 32) {
        memcpy(tmp, vdm, size);
        if (tmp[2] == 0x01 && tmp[3] == 0x00) {
            tmp[3] = 0x40;
            return interface_send_msg_timeout(TYPE_VDM, tmp, size,
                                              INTERFACE_TIMEOUT);
        }
    }
    return 1;
}

u8 send_svid(const u8 *svid, u8 size)
{
    u8 tmp[4] = {
        0
    };
    if (NULL == svid || size != 4)
        return CMD_FAIL;
    memcpy(tmp, svid, size);
    return interface_send_msg_timeout(TYPE_SVID, tmp, size,
                                      INTERFACE_TIMEOUT);
}

u8 send_rdo(const u8 *rdo, u8 size)
{
    u8 i;
    if (NULL == rdo)
        return CMD_FAIL;
    if ((size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
        (size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
        return CMD_FAIL;
    }
    for (i = 0; i < size; i++)
        pd_rdo[i] = *rdo++;

    return interface_send_msg_timeout(TYPE_PWR_OBJ_REQ, pd_rdo, size,
                                      INTERFACE_TIMEOUT);
}

u8 send_power_swap(void)
{
    return interface_pr_swap();
}

u8 send_data_swap(void)
{
    return interface_dr_swap();
}

u8 send_accept(void)
{
    return interface_send_msg_timeout(TYPE_ACCEPT, 0, 0, INTERFACE_TIMEOUT);
}

u8 send_reject(void)
{
    return interface_send_msg_timeout(TYPE_REJECT, 0, 0, INTERFACE_TIMEOUT);
}

u8 send_soft_reset(void)
{
    return interface_send_soft_rst();
}

u8 send_hard_reset(void)
{
    return interface_send_hard_rst();
}

char *interface_to_str(unsigned char header_type)
{
    return (header_type == TYPE_PWR_SRC_CAP) ? "src cap" :
           (header_type == TYPE_PWR_SNK_CAP) ? "snk cap" :
           (header_type == TYPE_PWR_OBJ_REQ) ? "RDO" :
           (header_type == TYPE_DP_SNK_IDENTITY) ? "snk identity" :
           (header_type == TYPE_SVID) ? "svid" :
           (header_type == TYPE_PSWAP_REQ) ? "PR_SWAP" :
           (header_type == TYPE_DSWAP_REQ) ? "DR_SWAP" :
           (header_type == TYPE_GOTO_MIN_REQ) ? "GOTO_MIN" :
           (header_type == TYPE_DP_ALT_ENTER) ? "DPALT_ENTER" :
           (header_type == TYPE_DP_ALT_EXIT) ? "DPALT_EXIT" :
           (header_type == TYPE_VCONN_SWAP_REQ) ? "VCONN_SWAP" :
           (header_type == TYPE_GET_DP_SNK_CAP) ? "GET_SINK_DP_CAP" :
           (header_type == TYPE_DP_SNK_CFG) ? "dp cap" :
           (header_type == TYPE_SOFT_RST) ? "Soft Reset" :
           (header_type == TYPE_HARD_RST) ? "Hard Reset" :
           (header_type == TYPE_RESTART) ? "Restart" :
           (header_type == TYPE_PD_STATUS_REQ) ? "PD Status" :
           (header_type == TYPE_ACCEPT) ? "ACCEPT" :
           (header_type == TYPE_REJECT) ? "REJECT" :
           (header_type == TYPE_VDM) ? "VDM" :
           (header_type ==
            TYPE_RESPONSE_TO_REQ) ? "Response to Request" : "Unknown";
}

inline unsigned char cac_checksum(unsigned char *pSendBuf, unsigned char len)
{
    unsigned char i;
    unsigned char checksum;
    checksum = 0;
    for (i = 0; i < len; i++)
        checksum += *(pSendBuf + i);

    return (u8) (0 - checksum);
}

void printb(const char *buf, size_t size)
{
#ifdef OCM_DEBUG
    while (size--)
        printk("%0x ", *buf++);
    printk("\n");
#endif
}

void interface_init(void)
{
    pbuf_rx_front = 0;
    pbuf_tx_rear = 0;
    downstream_pd_cap = 0;
}

void send_initialized_setting(void)
{
    unsigned char send_init_setting_state,c;


    send_init_setting_state=1;

    do {
        switch(send_init_setting_state) {
        default:
        case 0:
            break;
        case 1:
            
            send_pd_msg(TYPE_PWR_SRC_CAP, (const char *)init_src_caps,  sizeof(init_src_caps));
            send_init_setting_state++;
            break;
        case 2:
            device_addr= OCM_SLAVE_I2C_ADDR2;
            c= ReadReg( InterfaceSendBuf_Addr);
            device_addr= OCM_SLAVE_I2C_ADDR1;
            if (c==0) {
                send_init_setting_state++;
            } else
                break;
        case 3:
            
            send_pd_msg(TYPE_PWR_SNK_CAP, (const char *)init_snk_cap,
                        sizeof(init_snk_cap));
            send_init_setting_state++;
            break;
        case 4:
            device_addr= OCM_SLAVE_I2C_ADDR2;
            c= ReadReg( InterfaceSendBuf_Addr);
            device_addr= OCM_SLAVE_I2C_ADDR1;
            if (c==0) {
                send_init_setting_state++;
            } else
                break;
        case 5:
            
            send_pd_msg(TYPE_DP_SNK_IDENTITY, init_snk_ident,
                        sizeof(init_snk_ident));
            send_init_setting_state++;
            break;
        case 6:
            device_addr= OCM_SLAVE_I2C_ADDR2;
            c= ReadReg( InterfaceSendBuf_Addr);
            if (c==0) {
                send_init_setting_state++;
            } else
                break;
        case 7:
            
            send_pd_msg(TYPE_SVID, init_svid, sizeof(init_svid));
            send_init_setting_state++;
            break;
        case 8:
        case 9:
            send_init_setting_state = 0;
            break;
        }
    } while(send_init_setting_state!=0);

}

void chip_register_init(void)
{
#ifdef PD_CTS_TEST
    

    

    
#else
    WriteReg(TRY_UFP_TIMER, 0x96);
#endif

#ifdef SUP_TRY_SRC_SINK
    WriteReg(VBUS_DELAY_TIME, 0x19);
#endif

#ifdef SUP_INT_VECTOR
    
    WriteReg(INTERFACE_INTR_MASK, INTR_MASK_SETTING);
#else
    WriteReg(INTERFACE_INTR_MASK, 0xff);
#endif

#ifdef AUTO_RDO_ENABLE
    WriteReg(AUTO_PD_MODE, ReadReg(AUTO_PD_MODE) | AUTO_PD_ENABLE);
    
	
	WriteReg(MAX_VOLTAGE_SETTING, 0x5A);  
    
    
	
	
	WriteReg(MAX_POWER_SETTING, 0x36);	 
    
    
    WriteReg(MIN_POWER_SETTING, 0x002);  
#endif

	WriteReg(AUTO_PD_MODE, ReadReg(AUTO_PD_MODE) | TRY_SNK_ENABLE);
}

inline void reciever_reset_queue(void)
{
    rx_buf_front() = rx_buf_rear();
    WriteReg(RX_BUF_FRONT, rx_buf_front());
}

#if 0
#define STS_DATA_ROLE_CHANGE (((sys_status&DATA_ROLE)!=(sys_sta_bak&DATA_ROLE)) ? DATA_ROLE_CHANGE:0)
#define STS_VCONN_CHANGE (((sys_status&VCONN_STATUS)!=(sys_sta_bak&VCONN_STATUS)) ? VCONN_CHANGE:0)
#define STS_VBUS_CHANGE (((sys_status&VBUS_STATUS)!=(sys_sta_bak&VBUS_STATUS)) ? VBUS_CHANGE:0)
void __maybe_unused handle_intr_vector(void)
{
    static unsigned char sys_sta_bak = 0;
    unsigned char sys_status;
    u8 intr_vector = ReadReg(INTERFACE_CHANGE_INT);
    u8 status;

#ifdef OCM_DEBUG
    pr_info(" intr vector = 0x%02x\n",  intr_vector);
#endif
    WriteReg(INTERFACE_CHANGE_INT, intr_vector & (~intr_vector));
    if ((~INTR_MASK_SETTING) & intr_vector & RECEIVED_MSG)
        polling_interface_msg(INTERACE_TIMEOUT_MS);
    if ((~INTR_MASK_SETTING) & intr_vector & CC_STATUS_CHANGE) {
        status = ReadReg(NEW_CC_STATUS);
        
		anx7688_cc_change(status);
    }
	if (atomic_read(&anx7688_power_status) != 1)
		return;
    sys_status = ReadReg(SYSTEM_STSTUS);

    if ((~INTR_MASK_SETTING) & ((intr_vector & VBUS_CHANGE) | STS_VBUS_CHANGE)) {
        status = ReadReg(SYSTEM_STSTUS);
        pd_vbus_control_default_func(status & VBUS_STATUS);
    }
    if ((~INTR_MASK_SETTING) & ((intr_vector & VCONN_CHANGE) | STS_VCONN_CHANGE)) {
        status = ReadReg(SYSTEM_STSTUS);
        pd_vconn_control_default_func(status & VCONN_STATUS);
    }
    if ((~INTR_MASK_SETTING) & ((intr_vector & DATA_ROLE_CHANGE) | STS_DATA_ROLE_CHANGE)) {
        status = ReadReg(SYSTEM_STSTUS);
        pd_drole_change_default_func(status & DATA_ROLE);
    }

    sys_sta_bak = sys_status;

    clear_soft_interrupt();


}
#endif

#define TX_ROUTINE_CHECK() do {\
	if (time_before(expire, jiffies)) {\
		pr_info("TX Timeout %d\n", msg_total_len);\
		return CMD_FAIL;\
	}	\
} while (0)
#define RX_ROUTINE_CHECK() do {\
	if (time_before(expire, jiffies)) {\
		goto err_timeout;\
	}	\
} while (0)




u8 interface_send_msg_timeout(u8 type, u8 *pbuf, u8 len, int timeout_ms)
{
    unsigned char c,sending_len;
    unsigned char WriteDataBuf[32];


    
    WriteDataBuf[0] = len + 1;	
    WriteDataBuf[1] = type;
    memcpy(WriteDataBuf + 2, pbuf, len);
    
    WriteDataBuf[len + 2] = cac_checksum(WriteDataBuf, len + 1 + 1);



    sending_len = WriteDataBuf[0] + 2;
    device_addr= OCM_SLAVE_I2C_ADDR2;


    c=ReadReg( InterfaceSendBuf_Addr);
    if (c==0) {
        WriteBlockReg( InterfaceSendBuf_Addr , sending_len, &WriteDataBuf[0]);
    } else {
        pr_info("Tx Buf Full\n");
    }
    device_addr= OCM_SLAVE_I2C_ADDR1;

    return 0;

}

u8 polling_interface_msg(int timeout_ms)
{
    unsigned char ReadDataBuf[32];
    unsigned char global_i,checksum ;
    extern unsigned char device_addr;

    device_addr= OCM_SLAVE_I2C_ADDR2;

    ReadBlockReg( InterfaceRecvBuf_Addr, 32, (unsigned char *)ReadDataBuf);

    if (ReadDataBuf[0]!=0) {

        WriteReg(InterfaceRecvBuf_Addr,0);

        device_addr= OCM_SLAVE_I2C_ADDR1;

        checksum = 0;
        for(global_i = 0; global_i < ReadDataBuf[0] + 2; global_i++) {
            checksum += ReadDataBuf[global_i];
        }
        if(checksum == 0) {

            pr_info("\n>>%s\n", interface_to_str(ReadDataBuf[1]));
            dispatch_rcvd_pd_msg((PD_MSG_TYPE) ReadDataBuf[1], &(ReadDataBuf[2]), ReadDataBuf[0] - 1);
            return CMD_SUCCESS;


        } else {
            pr_info("checksum error! n");
            return CMD_FAIL;
        }

    }
    else
    {
        device_addr= OCM_SLAVE_I2C_ADDR1;
    }

    return CMD_FAIL;

}

#define MAX_REQUEST_VOLTAGE 5000
#define MAX_REQUEST_CURRENT 3000
#define set_rdo_value(v0, v1, v2, v3)	\
	do {				\
		pd_rdo[0] = (v0);	\
		pd_rdo[1] = (v1);	\
		pd_rdo[2] = (v2);	\
		pd_rdo[3] = (v3);	\
	} while (0)

u8 sel_voltage_pdo_index = 0x02;
u8 build_rdo_from_source_caps(u8 obj_cnt, u8 *buf)
{
	u8 i = 0;
	u16 pdo_h, pdo_l, pdo_h_tmp, pdo_l_tmp;
	u16 max_request_ma;
	u32 pdo_max, pdo_max_tmp;

	struct htc_pd_data pdo_data;
	int device_max_ma = 0;

	pdo_max = 0;
	obj_cnt &= 0x07;
	obj_cnt = (obj_cnt > 6) ? 6 : obj_cnt; 

	
	for (i = 0; i < obj_cnt; i++) {
		pdo_l_tmp = buf[i * 4 + 0];
		pdo_l_tmp |= (u16) buf[i * 4 + 1] << 8;
		pdo_h_tmp = buf[i * 4 + 2];
		pdo_h_tmp |= (u16) buf[i * 4 + 3] << 8;

		
		pdo_max_tmp =
			(u16) (((((pdo_h_tmp & 0xf) << 6) | (pdo_l_tmp >> 10)) &
					0x3ff) * 50);
		pdo_data.pd_list[i][0] = pdo_max_tmp;	
		pdo_data.pd_list[i][1] = (int)((pdo_l_tmp & 0x3ff) * 10);  

#if 0 
		if (pdo_max_tmp > pdo_max) {
			pdo_max = pdo_max_tmp;
			pdo_l = pdo_l_tmp;
			pdo_h = pdo_h_tmp;
			sel_voltage_pdo_index = i;
		}
#endif
	}

	
	sel_voltage_pdo_index = (u8) htc_battery_pd_charger_support(obj_cnt,
								pdo_data, &device_max_ma);
	pr_info("device_max_ma=%d, cnt %d index %d\n", device_max_ma, obj_cnt,
								sel_voltage_pdo_index);
	if (sel_voltage_pdo_index < 0) {
		pr_info("RDO Mismatch !!!\n");
		set_rdo_value(0x0A, 0x28, 0x00, 0x10);
		return 0;
	}

	pdo_l = buf[sel_voltage_pdo_index * 4 + 0];
	pdo_l |= (u16) buf[sel_voltage_pdo_index * 4 + 1] << 8;
	pdo_h = buf[sel_voltage_pdo_index * 4 + 2];
	pdo_h |= (u16) buf[sel_voltage_pdo_index * 4 + 3] << 8;

	max_request_ma = device_max_ma;

	
	
	if ((pdo_h & (3 << 14)) != (PDO_TYPE_BATTERY >> 16)) {
		usb_set_dwc_property(PROPERTY_CURRENT_MAX, (unsigned int)max_request_ma * 1000);
		
		
		if (pdo_data.pd_list[sel_voltage_pdo_index][1] < 1000) {
			pdo_max = RDO_FIXED(sel_voltage_pdo_index + 1,
								max_request_ma,
								MAX_REQUEST_CURRENT, 0);
			pdo_max |= RDO_CAP_MISMATCH;
			set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
							(pdo_max >> 16) & 0xff,
							(pdo_max >> 24) & 0xff);
			pr_info("FIXED PDO: selected current value < 1000mA, +cap_mismatch\n");
			return 1;
		}
		else {	
			pdo_max = RDO_FIXED(sel_voltage_pdo_index + 1,
								max_request_ma,
								max_request_ma,
								0);
			set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
							(pdo_max >> 16) & 0xff,
							(pdo_max >> 24) & 0xff);
			pr_info("FIXED PDO: selected current value >= 1000mA, send to PD charger\n");
			return 1;
		}
	}
	else {	
		usb_set_dwc_property(PROPERTY_CURRENT_MAX, MAX_REQUEST_CURRENT * 1000);
		pdo_max = RDO_FIXED(sel_voltage_pdo_index + 1,
							MAX_REQUEST_CURRENT,
							MAX_REQUEST_CURRENT, 0);
		set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
					(pdo_max >> 16) & 0xff, (pdo_max >> 24) & 0xff);
		return 1;
	}

#if 0 
#ifdef OCM_DEBUG
    pr_info("maxV=%d, cnt %d index %d\n", pdo_max_tmp, obj_cnt,
            sel_voltage_pdo_index);
#endif
    if ((pdo_h & (3 << 14)) != (PDO_TYPE_BATTERY >> 16)) {
        max_request_ma = (u16) ((pdo_l & 0x3ff) * 10);
#ifdef OCM_DEBUG
        pr_info("maxMa %d\n", max_request_ma);
#endif
        
        if (max_request_ma < MAX_REQUEST_CURRENT) {
            pdo_max =
                RDO_FIXED(sel_voltage_pdo_index + 1, max_request_ma,
                          max_request_ma, 0);
            pdo_max |= RDO_CAP_MISMATCH;
            set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
                          (pdo_max >> 16) & 0xff,
                          (pdo_max >> 24) & 0xff);
            return 1;
        } else {
            pdo_max =
                RDO_FIXED(sel_voltage_pdo_index + 1,
                          MAX_REQUEST_CURRENT, MAX_REQUEST_CURRENT,
                          0);
            set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
                          (pdo_max >> 16) & 0xff,
                          (pdo_max >> 24) & 0xff);

            return 1;
        }
    } else {
        pdo_max =
            RDO_FIXED(sel_voltage_pdo_index + 1, MAX_REQUEST_CURRENT,
                      MAX_REQUEST_CURRENT, 0);
        set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
                      (pdo_max >> 16) & 0xff, (pdo_max >> 24) & 0xff);
        return 1;
    }

    pr_info("RDO Mismatch !!!\n");
    set_rdo_value(0x0A, 0x28, 0x00, 0x10);

    return 0;
#endif
}

u32 change_bit_order(u8 *pbuf)
{
    return ((u32)pbuf[3] << 24) | ((u32)pbuf[2] << 16)
           | ((u32)pbuf[1] << 8) | pbuf[0];
}
u8 pd_check_requested_voltage(u32 rdo)
{
    int max_ma = rdo & 0x3FF;
    int op_ma = (rdo >> 10) & 0x3FF;
    int idx = rdo >> 28;

    u32 pdo;
    u32 pdo_max;

    if (!idx || idx > pd_src_pdo_cnt) {
        pr_info("rdo = %x, Requested RDO is %d, Provided RDO number is %d\n", rdo, (unsigned int)idx, (unsigned int)pd_src_pdo_cnt);
        return 0; 
    }
    
    pdo = change_bit_order(pd_src_pdo + ((idx - 1) * 4));
    pdo_max = (pdo & 0x3ff);
#ifdef OCM_DEBUG
    pr_info("pdo_max = %x\n", pdo_max);
#endif
    
    
    if (op_ma > pdo_max)
        return 0; 
    if (max_ma > pdo_max)
        return 0; 



    return 1;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_source_caps_default_callback(void *para, u8 para_len)
{
    u8 *pdo = 0;
    u8 ret = 1;
    pdo = (u8 *) para;
    if (para_len % 4 != 0)
        return ret;
    if (build_rdo_from_source_caps(para_len / 4, para)) {
        ret = interface_send_request();
        pr_info("Snd RDO %x %x %x %x succ\n", pd_rdo[0], pd_rdo[1],
                pd_rdo[2], pd_rdo[3]);
    }
    return ret;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
   * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_sink_caps_default_callback(void *para, u8 para_len)
{
    u8 *pdo = 0;

    pdo = (u8 *) para;
    if (para_len % 4 != 0)
        return 0;
    if (para_len > VDO_SIZE)
        return 0;
    
    return 1;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_pwr_object_req_default_callback(void *para, u8 para_len)
{
    u8 *pdo = (u8 *) para;
    u8 ret = 1;
    u32 rdo = 0;
    if (para_len != 4)
        return ret;

    rdo = pdo[0] | (pdo[1] << 8) | (pdo[2] << 16) | (pdo[3] << 24);
    if (pd_check_requested_voltage(rdo))
        ret = send_accept();
    else
        ret = interface_send_reject();

    return ret;
}

/* Recieve accept message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_accept_default_callback(void *para, u8 para_len)
{

    return 1;
}

/* Recieve reject message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_reject_default_callback(void *para, u8 para_len)
{

    return 1;
}

/* Recieve reject message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_goto_min_default_callback(void *para, u8 para_len)
{

    return 1;
}

#if 0
void __maybe_unused pd_vbus_control_default_func(bool on)
{
#ifdef OCM_DEBUG
    pr_info("=====vbus control %d\n", (int)on);
#endif
#ifdef SUP_VBUS_CTL
    anx7688_vbus_control(on);
#else
	if (on) {
		anx7688_set_prop(ANX_PROLE, PR_SOURCE);
		anx7688_set_prop(ANX_PMODE, MODE_DFP);
		dwc3_pd_vbus_ctrl(1);
	}
	else {
		anx7688_set_prop(ANX_PROLE, PR_SINK);
		anx7688_set_prop(ANX_PMODE, MODE_UFP);
		dwc3_pd_vbus_ctrl(0);
	}
	dual_role_instance_changed(ohio_get_dual_role_instance()); 
#endif

}

void __maybe_unused pd_vconn_control_default_func(bool on)
{
    
	anx7688_platform_vconn_ctl(on);
}

void __maybe_unused pd_cc_status_default_func(u8 cc_status)
{
    
#ifdef OCM_DEBUG
    pr_info("cc status %x\n", cc_status);
#endif
	if (cc_status == 0x22) {
		anx7688_audio_accessory(1);
	}
}

void __maybe_unused pd_drole_change_default_func(bool on)
{
    
#ifdef OCM_DEBUG
	pr_info("data role control %d\n", (int)on);
#endif

	anx7688_enable_drole(on);
}
#endif

/* Recieve comand response message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  *  void *para :
  *   para_len :
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_cmd_rsp_default_callback(void *para, u8 para_len)
{
    u8 pd_cmd, pd_response;

    pd_cmd =  *(u8 *)para;
    pd_response = *((u8 *)para + 1);

    switch (pd_cmd) {
    case TYPE_PWR_OBJ_REQ:

        if (pd_response == CMD_SUCCESS)
            pr_info("pd_cmd RDO request result is successful\n");
        else if (pd_response == CMD_REJECT)
            pr_info("pd_cmd RDO reques result is rejected\n");
        else if (pd_response == CMD_BUSY)
            pr_info("pd_cmd RDO reques result is busy\n");
        else if (pd_response == CMD_FAIL)
            pr_info("pd_cmd RDO reques result is fail\n");
        else
            pr_info("pd_cmd RDO reques result is unknown\n");
        break;
    case TYPE_VCONN_SWAP_REQ:

        if (pd_response == CMD_SUCCESS)
            pr_info("pd_cmd VCONN Swap result is successful\n");
        else if (pd_response == CMD_REJECT)
            pr_info("pd_cmd VCONN Swap result is rejected\n");
        else if (pd_response == CMD_BUSY)
            pr_info("pd_cmd VCONN Swap result is busy\n");
        else if (pd_response == CMD_FAIL)
            pr_info("pd_cmd VCONN Swap result is fail\n");
        else
            pr_info("pd_cmd VCONN Swap result is unknown\n");
        break;
    case TYPE_DSWAP_REQ:

        if (pd_response == CMD_SUCCESS)
            pr_info("pd_cmd DRSwap result is successful\n");
        else if (pd_response == CMD_REJECT)
            pr_info("pd_cmd DRSwap result is rejected\n");
        else if (pd_response == CMD_BUSY)
            pr_info("pd_cmd DRSwap result is busy\n");
        else if (pd_response == CMD_FAIL)
            pr_info("pd_cmd DRSwap result is fail\n");
        else
            pr_info("pd_cmd DRSwap result is unknown\n");
        break;
    case TYPE_PSWAP_REQ:

        if (pd_response == CMD_SUCCESS)
            pr_info("pd_cmd PRSwap result is successful\n");
        else if (pd_response == CMD_REJECT)
            pr_info("pd_cmd PRSwap result is rejected\n");
        else if (pd_response == CMD_BUSY)
            pr_info("pd_cmd PRSwap result is busy\n");
        else if (pd_response == CMD_FAIL)
            pr_info("pd_cmd PRSwap result is fail\n");
        else
            pr_info("pd_cmd PRSwap result is unknown\n");
        break;
    default:
        break;
    }

    return CMD_SUCCESS;
}

u8 recv_pd_hard_rst_default_callback(void *para, u8 para_len)
{
#ifdef OCM_DEBUG
    pr_info("recv pd hard reset\n");
#endif

    return CMD_SUCCESS;
}

/* Recieve Data Role Swap message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through init_pd_msg_callback, it it pd_callback is not 0, using the default
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_dswap_default_callback(void *para, u8 para_len)
{
    
    return 1;
}

/* Recieve Power Role Swap message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through init_pd_msg_callback, it it pd_callback is not 0, using the default
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_pswap_default_callback(void *para, u8 para_len)
{
    
    return 1;
}
static pd_callback_t pd_callback_array[256] = { 0 };

pd_callback_t get_pd_callback_fnc(PD_MSG_TYPE type)
{
    pd_callback_t fnc = 0;
    if (type < 256)
        fnc = pd_callback_array[type];
    return fnc;
}

void set_pd_callback_fnc(PD_MSG_TYPE type, pd_callback_t fnc)
{
    pd_callback_array[type] = fnc;
}

void init_pd_msg_callback(void)
{
    u8 i = 0;
    for (i = 0; i < 256; i++)
        pd_callback_array[i] = 0x0;
}
