#include "udp.h"
#include "lwip.h"
#include "udp_server.h"

#define UDP_LOCAL_PORT     8880 


static void udp_server_receive_callback(void *arg, struct udp_pcb *upcb,
    struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    uint32_t i;
    
    printf("get msg from %d:%d:%d:%d port:%d:\r\n",
        *((uint8_t *)&addr->addr), *((uint8_t *)&addr->addr + 1),
        *((uint8_t *)&addr->addr + 2), *((uint8_t *)&addr->addr + 3), port);
    
    if (p != NULL)
    {
        struct pbuf *ptmp = p;
        
        while(ptmp != NULL)
        {
            for (i = 0; i < p->len; i++)
            {
                printf("%c", *((char *)p->payload + i));
            }
            udp_sendto(upcb, p, addr, port);
            
            ptmp = p->next;
        }
        
        
        printf("\r\n");
    }
    
    pbuf_free(p);
}


void udp_server_init(void)
{
    struct udp_pcb *upcb;
    err_t err;

    upcb = udp_new();

    if (upcb)
    {
        err = udp_bind(upcb, &ipaddr, UDP_LOCAL_PORT);

        if(err == ERR_OK)
        {
            udp_recv(upcb, udp_server_receive_callback, NULL);
        }
        else
        {
            udp_remove(upcb);
            
            printf("can not bind pcb\r\n");
        }
    }
}