#include "tcp_server.h"

#include "lwip.h"
#include "tcp.h"

#define TCP_LOCOL_ADDR	9000


/*  Callback function when data receive */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb,
                             struct pbuf *p, err_t err)
{
    uint32_t i;
    
    //tcp_write(tpcb, p->payload, p->len, 1);
    
    if (p != NULL)
    {
        struct pbuf *ptmp = p;
        
        printf("get msg from %d:%d:%d:%d port:%d:\r\n",
            *((uint8_t *)&tpcb->remote_ip.addr),
            *((uint8_t *)&tpcb->remote_ip.addr + 1),
            *((uint8_t *)&tpcb->remote_ip.addr + 2),
            *((uint8_t *)&tpcb->remote_ip.addr + 3),
            tpcb->remote_port);
        
        while(ptmp != NULL)
        {
            for (i = 0; i < p->len; i++)
            {
                printf("%c", *((char *)p->payload + i));
            }
            
            ptmp = p->next;
        }
        
        printf("\r\n");
        
				tcp_write(tpcb, p->payload, p->len, 1);
				
				/* Send ACK */
        tcp_recved(tpcb, p->tot_len);
        
        pbuf_free(p);
    }
    else if (err == ERR_OK)
    {
        printf("tcp client closed\r\n");
        
        tcp_recved(tpcb, p->tot_len);
        
        return tcp_close(tpcb);
    }

    return ERR_OK;
}


/* Callback function when new connect comes */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    printf("tcp client connected\r\n");
    
    printf("ip %d:%d:%d:%d port:%d\r\n",
        *((uint8_t *)&newpcb->remote_ip.addr),
        *((uint8_t *)&newpcb->remote_ip.addr + 1),
        *((uint8_t *)&newpcb->remote_ip.addr + 2),
        *((uint8_t *)&newpcb->remote_ip.addr + 3),
        newpcb->remote_port);
    
    tcp_write(newpcb, "tcp client connected\n", strlen("tcp client connected\n"), 0);
    
    tcp_recv(newpcb, tcp_server_recv);

    return ERR_OK;
}

void tcp_server_init(void){
	struct tcp_pcb *tpcb = NULL;
	
	tpcb = tcp_new();
	
	if(tpcb != NULL){
		err_t err;
		
		err = tcp_bind(tpcb, &ipaddr, TCP_LOCOL_ADDR);
		
		if(err == ERR_OK){
			tpcb = tcp_listen(tpcb);
			/* Register the TCP client accept callback function */
			tcp_accept(tpcb, tcp_server_accept);
			
			printf("tcp server listening\r\n");
			printf("tcp server ip:%d:%d:%d:%d prot:%d\r\n",
					*((uint8_t *)&ipaddr.addr),
					*((uint8_t *)&ipaddr.addr + 1),
					*((uint8_t *)&ipaddr.addr + 2),
					*((uint8_t *)&ipaddr.addr + 3),
					tpcb->local_port);
		}
	
	
	}

}
