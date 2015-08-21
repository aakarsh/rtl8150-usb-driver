#include <linux/init.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/usb.h>
#include <asm/uaccess.h>

#include "rtl8150.h"

//#define DRV_NAME "LDDA_USB"  // Use it to change name of interface from eth
static const char driver_name [] = "an-rtl8150";

static struct usb_device_id rtl8150_table[] ={ 
  {USB_DEVICE(VENDOR_ID, DEVICE_ID)},{}
};
MODULE_DEVICE_TABLE(usb, rtl8150_table);

//Device operations
static int rtl8150_probe(struct usb_interface *intf,
                         const struct usb_device_id *id);
static void rtl8150_disconnect(struct usb_interface *intf);
static int rtl8150_open(struct net_device *dev);
static int rtl8150_close(struct net_device *dev);
static int rtl8150_start_xmit(struct sk_buff *skb, struct net_device *dev);
static struct net_device_stats* rtl8150_get_stats(struct net_device *dev);

//device callbacks 
static void interrupt_callback(struct urb *urb);
static void read_bulk_callback(struct urb *urb);
static void write_bulk_callback(struct urb *urb);

//Register operations
static int read_register(struct usb_device* udev , __u16 reg, void* buf, int buf_len);
static int write_configuration_register(struct usb_device* udev , __u16 reg, u8 value);
static int write_register(struct usb_device* udev , __u16 reg, void* buf, int buf_len);

//proc support
static int rtl8150_stats_proc_show(struct seq_file *m, void* v);

static int rtl8150_reset_device(struct usb_device* usb_dev);
static struct usb_driver rtl8150_driver = {
  .name =	driver_name,
  .id_table =	rtl8150_table,
  .probe =	rtl8150_probe,
  .disconnect =	rtl8150_disconnect
};

static struct net_device_ops rtl8150_netdev_ops = {
  .ndo_open               = rtl8150_open,
  .ndo_stop               = rtl8150_close,
  .ndo_get_stats          = rtl8150_get_stats,
  .ndo_start_xmit         = rtl8150_start_xmit
};

static
int rtl8150_stats_proc_open(struct inode *inode, struct file *file)
{
  return single_open(file, rtl8150_stats_proc_show, PDE_DATA(inode));
}

static const struct file_operations rtl8150_proc_fops = {
	.open		= rtl8150_stats_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static 
int rtl8150_probe(struct usb_interface *intf,
                         const struct usb_device_id *id)
{
  struct net_device *ndev;
  rtl8150_t *rdev;

  //Extract usb_device from the usb_interface structure 
  struct usb_device *udev = interface_to_usbdev(intf);

  printk(KERN_INFO "an_usb_rtl8150:probe called\n");

  // Use eth%d
  //TODO: alloc_netdev_mqs(sizeof_priv, "DRV_NAME%d", ether_setup, txqs, rxqs);
  ndev = alloc_etherdev(sizeof(rtl8150_t));
  if (!ndev)
    return -ENOMEM;

  //Get pointer to private data space in net_dev
  rdev = netdev_priv(ndev);

  //Save ptrs to usb_device and net_dev
  rdev->udev = udev;	
  rdev->netdev = ndev;
  
  //Initialize rdev spin_lock
  spin_lock_init(&rdev->lock);

  //Set net_dev operations
  ndev->netdev_ops = &rtl8150_netdev_ops;

  //sysfs stuff. Sets up device link in /sys/class/net/interface_name 
  SET_NETDEV_DEV(ndev, &intf->dev);

   /**
   * You can stuff device private structure into USB Interface 
   * structure using usb_set_intfdata. That can be retrieved later 
   * using usb_get_drvdata, for example, in disconnect or other driver 
   * functions
   */
  //Save pointer to rdev in interface's device->device_data
  usb_set_intfdata(intf, rdev);

  rtl8150_reset_device(udev);

  ndev->dev_addr = kmalloc(sizeof(u8)*6,GFP_ATOMIC);

  if(!ndev->dev_addr)
    goto out;

  // receive the mac address into dev_addr
  read_register(udev,IDR,ndev->dev_addr,6);
  

  // Reusable usb request buffers
  rdev->tx_urb   = usb_alloc_urb(0,GFP_KERNEL);
  rdev->rx_urb   = usb_alloc_urb(0,GFP_KERNEL);
  rdev->intr_urb = usb_alloc_urb(0,GFP_KERNEL);


  /* Register netdevice. If fail goto out  */
  if (register_netdev(ndev) != 0) {
    printk(KERN_ALERT "error registering netdev\n");
    goto out;
  }	

  printk(KERN_INFO "an_usb_rtl8150: Read mac Address : [%pM] \n",
         rdev->netdev->dev_addr);
  printk(KERN_INFO "an_usb_rtl8150: Read mac Address : [%pM] \n",
         ndev->dev_addr);

  /* Length of Ethernet frame. It is a "hardware header length",
   * number of octets that lead the transmitted packet before IP
   * header, or other protocol information.  
   * Value is 14 for Ethernet interfaces.
   */
  ndev->hard_header_len = 14;
  rdev->counters.probe++;

  rdev->proc_stats = proc_create_data("rtl8150-usb-stats", 0, NULL, &rtl8150_proc_fops, rdev);

  return 0;

 out:
  usb_set_intfdata(intf, NULL);
  free_netdev(ndev);
  return -EIO;
}

static
int rtl8150_reset_device(struct usb_device *udev)
{
  u8  cr_value;
  // 1. Reset the device 
  write_configuration_register(udev,CR,CR_SOFT_RESET);
  //TODO cycling wait here
  udelay(600);

  // 2. Confirm reset has happened 
  read_register(udev,CR,&cr_value,1);

  printk(KERN_INFO "rtl8150_reset_device: cr_value: %x \n",cr_value);
  if((cr_value & 0x10) != 0){
    return -1;
  }  
  return 0;
}

static
int initiate_new_rx_request(rtl8150_t * rdev, int reuse_skb)
{
  int res;
  struct sk_buff *new_skb;

  if(!reuse_skb) {
    // Allocate a new skb 
    new_skb = netdev_alloc_skb(rdev->netdev, RTL8150_MTU +2);  
    // Align the skb
    skb_reserve(new_skb,2);
    // TODO some protection
    rdev->rx_skb = new_skb;
  }  

  usb_fill_int_urb(rdev->rx_urb,rdev->udev,
                   usb_rcvbulkpipe(rdev->udev,USB_EP_BULK_IN),
                   rdev->rx_skb->data,rdev->rx_skb->data_len,
                   read_bulk_callback,rdev,500);
  
  // Asynchronous submit urb wait for callback when packet arrives.
  res = usb_submit_urb(rdev->rx_urb,GFP_ATOMIC);
  
  if(res < 0) {
    printk(KERN_INFO "an_usb_rtl8150_open: bulk request send failure %d \n",res);
  }  
  return res;  
}

static 
int initial_interrupt_request(rtl8150_t *rdev)
{
  int res;  
  struct urb *urb = usb_alloc_urb(0,GFP_KERNEL);
  //rdev->intr_urb;         
  
  rdev->interrupt_buffer = kmalloc(8,GFP_ATOMIC);
  
  usb_fill_int_urb(urb,rdev->udev,
                   usb_rcvintpipe(rdev->udev, USB_EP_INT), 
                   rdev->interrupt_buffer,8, 
                   interrupt_callback,
                   rdev, /*context*/
                   500   /*ms*/); 


  res = usb_submit_urb(rdev->intr_urb, GFP_ATOMIC);  

  if(!urb || !urb->complete){
    printk(KERN_INFO "an_usb_rtl8150: initial_interrupt_request Complete flag not set \n");
  } 
  
  if(res < 0) { // TODO why is this failing seeing -22
    printk(KERN_INFO "an_usb_rtl8150: initial_interrupt_request: usb_submit_urb : %d \n", res);
  }

  printk(KERN_INFO "an_usb_rtl8150: initial_interrupt_request: submitted : %d \n", res);
  return res;
}

static int rtl8150_open(struct net_device *dev)
{

  struct rtl8150 *rdev = netdev_priv(dev);
  short cscr;
  printk(KERN_INFO "an-rtl8150_open \n");  
  rdev->counters.open++;

  // Transmission queue start 

  // Set the  devince address
  write_register(rdev->udev,IDR,dev->dev_addr,6);

  // 3. Initiate intterupt call back
  // Fetch 8 bytes for interrupt buffer.
  if(initial_interrupt_request(rdev) < 0) {
    printk(KERN_ALERT "an_usb_rtl8150_open: problem with interrupt request \n");
  }     

  // Initiate pending urb bulk to begin receiving
  if(initiate_new_rx_request(rdev,0)){
    printk(KERN_ALERT "an_usb_rtl8150_open: problem with bulk request send \n");    
  }


  // Reset the device to enable it
  rtl8150_reset_device(rdev->udev);
  write_configuration_register(rdev->udev ,RCR, RCR_DEFAULT);
  write_configuration_register(rdev->udev ,TCR, TCR_DEFAULT);
  write_configuration_register(rdev->udev ,CR, CR_DEFAULT);



  // Check carrier is present  
  read_register(rdev->udev , CSCR, &cscr, 2);
  printk(KERN_INFO "an-rtl8150_open : CSCR 0x%x\n",cscr);

  //TODO is this right
  if(cscr & CSCR_LINK_STATUS) {
    printk(KERN_INFO "an_usb_rtl8150_open: valid link detected \n");
    netif_carrier_on(dev);
  } else {
    printk(KERN_INFO "an_usb_rtl8150_open: no carrier link detected turn off netif_carrier \n");
    netif_carrier_off(dev);
  }
  
  //Force carrier on
  netif_carrier_on(dev);

  // Begin processing packets for transmission
  netif_start_queue(dev);



  return 0;
}


static
int rtl8150_close(struct net_device *dev)
{  
  struct rtl8150 *rdev = netdev_priv(dev);
  rdev->counters.close++;

  printk(KERN_INFO "an-rtl8150_close \n");

  // transmission queue stop
  // TODO ignore stop 
  //  netif_stop_queue(dev);
  return 0;
}


/**
 * Called when a usb bulk send has completed
 */
static void write_bulk_callback(struct urb *urb)
{
  rtl8150_t *rdev = urb->context;
  int status = urb->status;

  // Context not set for some reason
  if(!rdev) 
    return;

  rdev->counters.write_bulk_callback++;

  printk(KERN_INFO "an_usb_rtl8150: write_bulk_callback: skb len %d  tx-status: %d",
         rdev->tx_skb->len, status);

  // Always free the skb
  dev_kfree_skb_irq(rdev->tx_skb);

  // Keep track of last send
  rdev->netdev->trans_start = jiffies;

  // Restart device sending queue, allow flow control
  netif_wake_queue(rdev->netdev);
}

/**
 * Called from the upper layers to transmit a skb buffer.
 */
static int rtl8150_start_xmit(struct sk_buff *skb, 
                              struct net_device *ndev)
{
  rtl8150_t *rdev = netdev_priv(ndev);

  int res;
  printk(KERN_INFO "an-rtl8150_start_xmit packet-length :%d \n",
         skb->len);  

  //Temporarily suspend calls to send more data.
  netif_stop_queue(ndev);

  // Point transmit buffer to skb
  rdev->tx_skb = skb;

  // Create a bulk request to outbound end point.
  usb_fill_bulk_urb(rdev->tx_urb,rdev->udev,
                    usb_sndbulkpipe(rdev->udev,USB_EP_BULK_OUT),
                    skb->data,skb->len, write_bulk_callback ,rdev);

  // Submit async bluk send callback: write_bulk_callback
  res = usb_submit_urb(rdev->tx_urb,GFP_ATOMIC);

  if(!res){ // packet submitted, but not sent 

    // TODO maintain see we don't restart the net transmit 
    // queue instead 

  } else { // packet submit failure
    printk(KERN_INFO "an-rtl8150_start_xmit: Failure submitting usb packet \n");
    if(res!= -ENODEV) {
      netif_start_queue(ndev);
    }
  }
  return NETDEV_TX_OK;	
}

static struct net_device_stats* rtl8150_get_stats(struct net_device *dev)
{
  struct rtl8150 *rdev = netdev_priv( dev );
  rdev->counters.get_stats++;

  //  printk("an_usb_rtl8150: dev_get_stats: Add code later\n");  

  return &rdev->stats;
}

/**
 * USB disconnect routine - required else can't rmmod
 */
static
void rtl8150_disconnect(struct usb_interface *intf)
{  
  rtl8150_t *rdev = usb_get_intfdata(intf);
  
  rdev->counters.disconnect++;
  
  if(rdev) {
    proc_remove(rdev->proc_stats);
    unregister_netdev(rdev->netdev);

    if(rdev->tx_urb)
      usb_free_urb(rdev->tx_urb);
    if(rdev->rx_urb)
      usb_free_urb(rdev->rx_urb);
    if(rdev->intr_urb)
      usb_free_urb(rdev->intr_urb);

    free_netdev(rdev->netdev);
  }
}


/**
 * This is a callback function which will get called when a packet
 * packet has been received from the usb device.
 */
static
void read_bulk_callback(struct urb *urb)
{
  rtl8150_t *rdev = urb->context;  
  int status = urb->status;
  int pkt_len;
  int reuse_skb = 0 ;
  
  if((status == -75) && urb->actual_length == 0) { //resubmit request 
    rdev->counters.empty_msg_read_bulk_callback++;
    reuse_skb=1;
    goto resubmit;
  }
  
  //Why?
  pkt_len = urb->actual_length - 4;
  
  printk(KERN_INFO "an_usb_rtl8150: read_bulk_callback status %d urb-len %d packet length %d \n",
         status,urb->actual_length,pkt_len);

  rdev->counters.read_bulk_callback++;    

  if(urb->status > 0) {
  //Extend the data area of the skb to packet length
    skb_put(rdev->rx_skb, pkt_len);

    // Set up link layer protocal information
    rdev->rx_skb->protocol = eth_type_trans(rdev->rx_skb, rdev->netdev);

    //Pass skb up the protocal stack
    netif_rx(rdev->rx_skb);
    //TODO:update some statistics.
  }
  
 resubmit:
  if(initiate_new_rx_request(rdev,reuse_skb)){    
      printk("Error submitting urb from read callback\n");
  }
}


/**
 * Eight byte interrupt buffer contains :
 * 
 * +-----------------------------------------------------------------------+
 * | DATA0 | DATA1 | DATA2   | DATA3 | DATA4   | DATA5   | DATA6  | DATA7  | 
 * | TSR   | RSR   | GEP/MSR | WAKSR | NUMTXOK | RXLOST  | CRCERR | COLCNT | 
 * +-----------------------------------------------------------------------+
 * 
 * +---------------------------------------------------------------------------------------------------------------+
 * |               | 7       | 6           | 5      | 4        |  3       | 2        | 1             |  0          | 
 * | 00h TSR       | -       | -           | ECOL   | LCOL     | LOSS_CRS | JBR      | TX_BUF_EMPTY  | TX_BUF_FULL |
 * | 01h RSR       | -       | RX_BUF_FULL | LKCHG  | RUNT     | LONG     | CRC      | FAE           | ROK         |
 * | 02h GEP/MSR   | GEP1DAT | GEP0DAT     | -      | Duplex   | SPEED_100| LINK     | TXPF          | RXPF        |
 * | 03h WAKSR     | -       | PARM_EN     | -      | -        | WAKEUP_EV| LKWAKE_EV| MAGIC_EV      | BMU_EV      |
 * |---------------------------------------------------------------------------------------------------------------|
 * | 04  TXOK_CNT  | 8 bit counter valid packets transmitted                                                       | 
 * | 05h RXLOST_CNT| 8 bit counter packets lost in buffer overflow                                                 | 
 * | 06h CRCERR_CNT| 8 bit counter error packets                                                                   |
 * | 07h COL_CNT   | 8-big counter for collisions                                                                  |
 * +---------------------------------------------------------------------------------------------------------------+
 * 
 * Main call back to deal with various usb interrupts. This callback
 * is used mainlty to keep devince statistics and link status
 * 
 * 1. Check the status code of the interrupt requestt
 * 1.1. Error Types (-ECONNRESET , -ENONT, -ESHUTDOWN)
 * 2. Compute statistics
 *
 * Typical values :
 *
 *  TSR: 0x2        - TX_BUF_EMPTY 
 *  RSR: 0x4        - CRC
 *  MSR: 0x1c       - Duplix , SPEED_100, LINK
 *  WAKSR: 0x0      - 0 
 *  TXOK_CNT: 0x40  - 64
 *  RXLOST_CNT: 0x0 - 0 
 *
 */
static
void interrupt_callback(struct urb *urb)
{
  int res,status;
  u8* int_buf;
  
  typedef enum {tsr=0,rsr,msr,waksr,txok,rxlost,crcerr,colcnt} intr_indx_t;
  rtl8150_t *rdev = urb->context;  
  struct net_device* ndev = rdev->netdev;

  rdev->counters.interrupt_callback++;  

  status = urb->status;
  if(status < 0){
    printk(KERN_ALERT "an_usb_rtl8150: interrupt_callback error status : %d \n",status);
    goto resubmit;
  }

  int_buf = urb->transfer_buffer;

  if(int_buf[tsr] & TSR_ERRORS) {
    ndev->stats.tx_errors++;
    if((int_buf[tsr] & TSR_LCOL) || (int_buf[tsr] & TSR_ECOL)) {
      ndev->stats.collisions++;      
    }
  }

  // TODO : Work on MSR stuff.
  // Should we check so frequently 
  /*
  if(int_buf[msr] & MSR_LINK) {
    netif_carrier_on(rdev->netdev);
  } else {
    netif_carrier_off(rdev->netdev);
  }
  */
  //Resubmit requiest
 resubmit:
  res = usb_submit_urb(urb,GFP_ATOMIC);
  if(res < 0){
    printk(KERN_ALERT "Error resubmitting \n" );
  }
  
}


static
int read_register(struct usb_device* udev , __u16 reg, void* buf, int buf_len)
{
  return usb_control_msg(udev,usb_rcvctrlpipe(udev,USB_EP_CTRL),
                         RTL8150_REQ_GET_REGS,RTL8150_REQT_READ,
                         reg,0,
                         buf,buf_len,500);
}

static
int write_configuration_register(struct usb_device* udev , __u16 reg, u8 value)
{

  return write_register(udev,reg,&value,1);
}

static
int write_register(struct usb_device* udev , __u16 reg, void* buf, int buf_len)
{
  return usb_control_msg(udev,usb_sndctrlpipe(udev,USB_EP_CTRL),
                  RTL8150_REQ_SET_REGS,RTL8150_REQT_WRITE,
                  reg,0,buf,buf_len,500);          
}

static
int rtl8150_show_device_info(rtl8150_t *rdev , struct seq_file *m)
{
  seq_printf(m,"\nDevice Information : \n");
  seq_printf(m,"Interface Name : %s \n",rdev->netdev->name);
  seq_printf(m,"Mac Address : %pM \n",rdev->netdev->dev_addr);
  seq_printf(m,"Broadcast Address : %pM \n",rdev->netdev->broadcast);
  seq_printf(m,"MTU : %d \n",rdev->netdev->mtu);
  seq_printf(m,"Carrier Changes on<->off : %d \n", 
             atomic_read(&rdev->netdev->carrier_changes));
  return 0;
}

static
int rt18150_show_netstat(rtl8150_t *rdev , struct seq_file *m)
{
  struct net_device_stats* stats = &(rdev->netdev->stats);

  seq_printf(m,"\nDevice Network Statistics \n");
  seq_printf(m,"last_rx : %lu \n",rdev->netdev->last_rx);


  seq_printf(m,"rx_packets: %lu\n",stats->rx_packets);;
  seq_printf(m,"tx_packets: %lu\n",stats->tx_packets);;
  seq_printf(m,"rx_bytes: %lu\n",stats->rx_bytes);
  seq_printf(m,"tx_bytes: %lu\n",stats->tx_bytes);
  seq_printf(m,"rx_errors: %lu\n",stats->rx_errors);
  seq_printf(m,"tx_errors: %lu\n",stats->tx_errors);
  seq_printf(m,"rx_dropped: %lu\n",stats->rx_dropped);
  seq_printf(m,"tx_dropped: %lu\n",stats->tx_dropped);
  seq_printf(m,"multicast:; %lu\n",stats->multicast);
  seq_printf(m,"collisions:; %lu\n",stats->collisions);
  seq_printf(m,"rx_length: %lu\n",stats->rx_length_errors);
  seq_printf(m,"rx_over: %lu\n",stats->rx_over_errors);
  seq_printf(m,"rx_crc: %lu\n",stats->rx_crc_errors);
  seq_printf(m,"rx_frame: %lu\n",stats->rx_frame_errors);
  seq_printf(m,"rx_fifo: %lu\n",stats->rx_fifo_errors);
  seq_printf(m,"rx_missed: %lu\n",stats->rx_missed_errors);
  seq_printf(m,"tx_aborted: %lu\n",stats->tx_aborted_errors);
  seq_printf(m,"tx_carrier: %lu\n",stats->tx_carrier_errors);
  seq_printf(m,"tx_fifo: %lu\n",stats->tx_fifo_errors);
  seq_printf(m,"tx_heartbeat: %lu\n",stats->tx_heartbeat_errors);
  seq_printf(m,"tx_window: %lu\n",stats->tx_window_errors);
  seq_printf(m,"rx_compressed: %lu\n",stats->rx_compressed);
  seq_printf(m,"tx_compressed: %lu\n",stats->tx_compressed);
  return 0;
}

static
int rt18150_show_counters(rtl8150_t *rdev , struct seq_file *m)
{
  struct rtl8150_counters * c = &(rdev->counters);
  seq_printf(m,"\nDevice Method Counters \n");  
  seq_printf(m,"empty read callback: %lu \n",c->empty_msg_read_bulk_callback);
  seq_printf(m, "enable_net_traffic: %lu \n", c->enable_net_traffic);
  seq_printf(m,"interrupt_callback: %lu \n",c->interrupt_callback);
  seq_printf(m,"probe: %lu \n",c->probe);
  seq_printf(m,"open: %lu \n",c->open);
  seq_printf(m,"close: %lu \n",c->close);
  seq_printf(m,"disconnect: %lu \n",c->disconnect);
  seq_printf(m,"hardware_start: %lu  \n",c->hardware_start);
  seq_printf(m,"reset: %lu \n",c->reset);
  seq_printf(m,"start_xmit: %lu \n",c->start_xmit);
  seq_printf(m,"rx_submit_pending_request: %lu \n",c->rx_submit_pending_request);
  seq_printf(m,"get_stats: %lu \n",c->get_stats);
  seq_printf(m,"write_bulk_callback: %lu \n",c->write_bulk_callback);
  seq_printf(m,"read_bulk_callback: %lu \n",c->read_bulk_callback);
  return 0;
}

static
int rtl8150_stats_proc_show(struct seq_file *m, void* v)
{
  rtl8150_t *rdev = m->private;  
  rtl8150_show_device_info(rdev,m);
  rt18150_show_netstat(rdev,m);
  rt18150_show_counters(rdev,m);
  return 0;
}

module_usb_driver(rtl8150_driver);

MODULE_AUTHOR("Aakarsh Nair");
MODULE_DESCRIPTION("USB Driver for Realtek rtl8150 USB Ethernet Wired Card");
MODULE_LICENSE("Dual BSD/GPL");
  
