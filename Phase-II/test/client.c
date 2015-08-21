#include <stdio.h>
#include <pcap.h>

#define DEFAULT_INTERFACE "eth0"
#define BUF_SIZE 1500 
#define TIMEOUT   10000

static int packet_count = 0;


/**
 * Receive packet here and verify packet attributes
 * and the fact that it is the packet we were looking for.
 */
void got_packet(u_char *args, const struct pcap_pkthdr *header,
                const u_char *packet)
{

  packet_count++;
  printf("packet #%d !\n",packet_count);
  printf("Header Length : %d bytes \n", header->len);  

}

int main(int argc , char* argv[])
{
  //capture an ethernet packet from the interface.
  char* dev = argv[1];
  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_t * pdev;

  char* flt_exp = "host 10.0.0.20";
  struct bpf_program flt_prog ;
  bpf_u_int32 mask  ; 

  int ret;

  if(argc > 2){
    printf("Usage : client <interface> \n");
  } else{
    dev = DEFAULT_INTERFACE;
  }

  pdev = pcap_open_live(dev,BUF_SIZE,1,TIMEOUT,errbuf);
  if(!pdev) {
    printf("Error %s",errbuf);
    return -1;
  }

  if (pcap_datalink(pdev) != DLT_EN10MB) {
    fprintf(stderr, "Device %s doesn't provide Ethernet headers - not supported\n", dev);
    return -1;
  }

  ret = pcap_compile(pdev, &flt_prog,flt_exp, 0, mask);
  if(ret){
    fprintf(stderr,"Error creating filter  for expression %s : %d\n",flt_exp,ret);
    return -1;
  }

  ret = pcap_setfilter(pdev,&flt_prog);
  if(ret) { 
    fprintf(stderr,"Error setting filter  for expression %s : %d\n",flt_exp,ret);    
    return -1;
  }
  u_char * packet;
  struct pcap_pkthdr header;

  printf("Listening on device: %s\n",dev);

  ret = pcap_loop(pdev,0, &got_packet,NULL);

  if(ret) {
    fprintf(stderr,"Exiting packet capture : %d\n",ret);
  }

  printf("Exiting\n");
  return 0;
}
