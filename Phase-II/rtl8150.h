// rtl8150 Datasheet - Page 9, Vendor Specific Memory Read/Write Commands

/* Define these values to match your device */
#define	VENDOR_ID_REALTEK		0x0bda
#define PRODUCT_ID_RTL8150		0x8150

#define	RTL8150_MTU		1540

// See page 9 of rtl8150 data sheet
//BmReq
#define RTL8150_REQT_READ       0xc0
#define RTL8150_REQT_WRITE      0x40

//bReq
#define RTL8150_REQ_GET_REGS    0x05
#define RTL8150_REQ_SET_REGS    0x05


// Register offsets in device memory  - rtl8150 Datasheet page 17

/**
 * Ethernet address
 */
#define IDR     		0x0120  // MAC address is found here

/**
 * 
 * Command Register - 8 bit -  0x10
 * 
 * 0 - Autoload 
 * 1 - EP3CLREN
 * 2 - Transmit Enable
 * 3 - Recieve Enable
 * 4 - Soft Reset  -- SOFT RESET
 * 5 - WEPROM
 * 6 - Reserved
 * 7 - Reserved
 */

#define CR                      0x012e

/*  2,3 Transmit and Receive enable */
#define CR_DEFAULT               0x0c

/* Disable traffic */
#define  CR_TRAFFIC_DISABLE       0xf3
/**
 * Reset: Setting to 1 forces the RTL8150L(M) to a software reset state
 * which
 * 
 * 1. Disables the transmitter and receiver,
 * 2. Reinitializes the FIFOs, resets the system buffer pointer to the initial value
 *    Rx buffer is empty).
 * 4. The values of IDR0-5 and MAR0-7 will have no changes.
 * 
 * 5. This bit is 1 during the reset operation, and is cleared to 0 by the
 *    RTL8150L(M) when the reset operation is complete.
 */
/* bit 4*/
#define CR_SOFT_RESET           (1<<4) 

/**
 * Transmit configuration regsiter
 *  7,6 - Transmit retry multiples of 16
 *  5   - Reserved
 *  4,3 - Interframe gap time if below 9.6us for 10mbps 960 ns for 100 mbps
 *  2,1 - Reserved
 *  0   - No CRC
 */  
#define TCR                     0x012f

#define TCR_DEFAULT             0xd8
/**
 * Receive status register
 *  7 - Replace first two bytes CRC header
 *  6 - Accept CRC error packet
 *  5 - Accept (Runt < 64) byte packets
 *  4 - Accept Multicast packets
 *  3 - Accept Broadcast packets
 *  2 - Match receive address in MAC address
 *  1 - Accept ALL Multicast frames
 *  0 - Accept all Physical frames
 */
#define RCR                     0x0130
                               /* See  7,4,3,2,1 */
#define RCR_DEFAULT             0x9e

/**
   Medium status register
*/
#define MSR                     0x0137	  // carrier lost or ok
#define MSR_LINK                (1<<2)   // bit 2 in MSR register - page 29


/**
   pg 29. Data Sheet 

   CS configuration regsiter - 16 bit register - Used mostly for determining connection link status

   0:(0,rw):Bypass Scramble 
   1::Reserved
   2:(0,rw):Connection Status En Configures LED 1 to indicate connection status
   3:(0,ro):Connection Status : {1 - valid connected link detected,0 -disconnected link detected}
   4::Reserved
   5:(0,rw):F_Connect :  { 1 - Force connection, 0 - Disable } Diagnostic
   6:(1,rw):F_LINK_100 : {1 - Force 100 Mbs, 0 - Enabe }
   7:(1,rw):JBEN : { 1 - enable jabber , 0 - disable jabber }
   10-14 : Reserved
   15:(0,wo): Test fun
*/
#define CSCR                    0x014c   // Link status is found here
#define CSCR_LINK_STATUS        (1 << 3) // bit 3 in CSCR register - Page 29



#define INTBUFSIZE              8

/**
   Transmit status register
*/
#define TSR                     0x0132
#define TSR_ECOL                (1<<5)
#define TSR_LCOL                (1<<4)
#define TSR_LOSS_CRS            (1<<3)
#define TSR_JBR                 (1<<2)
#define TSR_ERRORS              (TSR_ECOL | TSR_LCOL | TSR_LOSS_CRS | TSR_JBR)

/**
   Receive status register errors 
*/
#define RSR_CRC                 (1<<2)
#define RSR_FAE                 (1<<1)
#define RSR_ERRORS              (RSR_CRC | RSR_FAE)

// Interrupt pipe data - for reporting errors and lost carrier

#define INT_TSR                 0x00
#define INT_RSR                 0x01
#define INT_MSR                 0x02
#define INT_WAKSR               0x03
#define INT_TXOK_CNT            0x04
#define INT_RXLOST_CNT          0x05
#define INT_CRERR_CNT           0x06
#define INT_COL_CNT             0x07

#define RTL8150_MTU             1540
#define RTL8150_TX_TIMEOUT      (HZ)

//Supported VenderID and DeviceID

#define VENDOR_ID       0x0bda          // Realtek Semiconductor Company
#define DEVICE_ID       0x8150          // RTL8150 USB Ethernet Controller

#define DRV_NAME "LDDA_USB"  // Use it to change name of interface from eth



#define USB_EP_CTRL      0       // Control Endpoint 
#define USB_EP_BULK_IN   1       // Bulk IN  Endpoint 
#define USB_EP_BULK_OUT  2       // Bulk OUT  Endpoint 
#define USB_EP_INT       3       // Bulk Interrupt  Endpoint 


// Represents offsets into 8-byte Interrupt End point
#define INT_TSR			0x00   // 5:ECOL, 4:LCOL, 3:LOSS_CRC, 2:JBR, 1:TX_BUF_EMPTY, 0:TX_BUF_FULL
#define INT_RSR			0x01   // 6:RX_BUF_FULL, 5:LKCHG, 4:RUNT, 3: LONG, 2:CRC , 1:FAE, 0:ROK
#define INT_MSR			0x02   // GEP/MSR 7:GEP1DAT, 6:GEP0DAT, ,4: Duplix, 3:SPEED-100,2:LINK,1:TXPF, 0:RXPF
#define INT_WAKSR		0x03   // 6:PARAM_EN, 3-0: WAKEUP_EV, LKWAKE_EV, MAGIC_EV, BMU_EV

//8-bit counters saturate to 255
#define INT_TXOK_CNT		0x04   // R 8-bit counter that counts for valid packets transmitted 
#define INT_RXLOST_CNT		0x05   // R 8-but counter that counts for packets lost due to Rx buffer overflow
#define INT_CRERR_CNT		0x06   // R 8-bit counter that counts for error packets
#define INT_COL_CNT		0x07   // R 8-bit counter that counts for collitions 


// Maintain method call statistics
struct rtl8150_counters {
  unsigned long enable_net_traffic;
  unsigned long interrupt_callback;
  unsigned long read_bulk_callback;
  unsigned long empty_msg_read_bulk_callback;
  unsigned long close;
  unsigned long disconnect;
  unsigned long hardware_start;
  unsigned long open;
  unsigned long probe;
  unsigned long reset;
  unsigned long start_xmit;
  unsigned long rx_submit_pending_request;
  unsigned long get_stats;
  unsigned long write_bulk_callback;
};
 
struct rtl8150 {

  struct usb_device *udev;           // USB to send requests too
  struct net_device *netdev;         // netdevice represented to upper layers
  struct net_device_stats stats;     // Net device stats
  struct rtl8150_counters counters;  //  custom counter statistics

  spinlock_t lock;                 

  struct urb *rx_urb;                // usb receive request buffer
  struct urb *tx_urb;                // usb transfer request buffer
  struct urb *intr_urb;              // usb intterupt request buffer
  struct urb *ctrl_urb;              // usb control   request buffer

  struct sk_buff *tx_skb;            // transfer skb buffer
  struct sk_buff *rx_skb;            // receive  skb buffer

  int intr_interval;                          // interfal to receive interrupts  
  void* interrupt_buffer;                     // pointer to interupt register register
  struct tasklet_struct rx_pending_tasklet;   // tasklet which wakes up and handles receiving packets

  struct proc_dir_entry* proc_stats;
};

typedef struct rtl8150 rtl8150_t;


// USB callback routines
static int rtl8150_probe(struct usb_interface *intf,
                         const struct usb_device_id *id);
static void rtl8150_disconnect(struct usb_interface *intf);
//static int rtl8150_suspend(struct usb_interface *intf, pm_message_t message);
//static int rtl8150_resume(struct usb_interface *intf);

static void read_bulk_callback(struct urb *urb);
static void write_bulk_callback(struct urb *urb);

// Net Device specific routines 
static int rtl8150_open(struct net_device *netdev);
static int rtl8150_close(struct net_device *netdev);
static int rtl8150_start_xmit(struct sk_buff *skb, struct net_device *netdev);
static struct net_device_stats* rtl8150_get_stats(struct net_device *netdev);

