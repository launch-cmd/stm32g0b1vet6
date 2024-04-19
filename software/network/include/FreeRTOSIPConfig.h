#ifndef FREERTOS_IP_CONFIG_H
#define FREERTOS_IP_CONFIG_H

// Constants Affecting the TCP/IP Stack Task Execution Behaviour
#define ipconfigEVENT_QUEUE_LENGTH 20
#define ipconfigIP_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define ipconfigIP_TASK_STACK_SIZE_WORDS (500)
#define ipconfig_PROCESS_CUSTOM_ETHERNET_FRAMES (0)
#define ipconfigUSE_NETWORK_EVENT_HOOK (0)

// Debug, Trace and Logging Settings
// #define ipconfigCHECK_IP_QUEUE_SPACE
// #define ipconfigHAS_DEBUG_PRINTF
// #define FreeRTOS_debug_printf
// #define ipconfigHAS_PRINTF
// #define FreeRTOS_printf
// #define ipconfigINCLUDE_EXAMPLE_FREERTOS_PLUS_TRACE_CALLS
// #define ipconfigTCP_IP_SANITY
// #define ipconfigTCP_MAY_LOG_PORT( x )
// #define ipconfigWATCHDOG_TIMER()
// #define ipconfigHAS_ROUTING_STATISTICS

// Hardware and Driver Specific Settings
// #define ipconfigBUFFER_PADDING
// #define ipconfigPACKET_FILLER_SIZE
#define ipconfigBYTE_ORDER (pdFREERTOS_LITTLE_ENDIAN)
#define ipconfigDRIVER_INCLUDED_RX_IP_CHECKSUM (0) //TODO: enable hardware RX checksum
#define ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM (0) //TODO: enable hardware TX checksum
#define ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES (0)
// #define ipconfigETHERNET_DRIVER_FILTERS_PACKETS
#define ipconfigETHERNET_MINIMUM_PACKET_BYTES (0)
#define ipconfigFILTER_OUT_NON_ETHERNET_II_FRAMES (0)
#define ipconfigNETWORK_MTU (1500)
#define ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS (ipconfigEVENT_QUEUE_LENGTH - 10)
// #define ipconfigUSE_LINKED_RX_MESSAGES 
#define ipconfigZERO_COPY_RX_DRIVER (0)
#define ipconfigZERO_COPY_TX_DRIVER (0)
#define ipconfigSUPPORT_NETWORK_DOWN_EVENT (1)

// TCP Specific Constants
#define ipconfigIGNORE_UNKNOWN_PACKETS (0)
#define ipconfigTCP_HANG_PROTECTION (1)
#define ipconfigTCP_HANG_PROTECTION_TIME (300)
#define ipconfigTCP_KEEP_ALIVE (1)
#define ipconfigTCP_KEEP_ALIVE_INTERVAL (1)
#define ipconfigTCP_MSS (1434)
#define ipconfigTCP_RX_BUFFER_LENGTH (4 * ipconfigTCP_MSS)
#define ipconfigTCP_TX_BUFFER_LENGTH (4 * ipconfigTCP_MSS)
// #define ipconfigTCP_TIME_TO_LIVE
#define ipconfigTCP_WIN_SEG_COUNT (256)
#define ipconfigUSE_TCP (1)
#define ipconfigUSE_TCP_TIMESTAMPS (0)
#define ipconfigUSE_TCP_WIN (1)
#define ipconfigTCP_SRTT_MINIMUM_VALUE_MS (1000)


// // UDP Specific Constants
// #define ipconfigUDP_MAX_RX_PACKETS
// #define ipconfigUDP_MAX_SEND_BLOCK_TIME_TICKS
// #define ipconfigUDP_PASS_ZERO_CHECKSUM_PACKETS
// #define ipconfigUDP_TIME_TO_LIVE

// // Other Constants Effecting Socket Behaviour
// #define ipconfigALLOW_SOCKET_SEND_WITHOUT_BIND
// #define ipconfigINCLUDE_FULL_INET_ADDR
// #define ipconfigSELECT_USES_NOTIFY
// #define ipconfigSOCK_DEFAULT_RECEIVE_BLOCK_TIME
// #define ipconfigSOCK_DEFAULT_SEND_BLOCK_TIME
// #define ipconfigSOCKET_HAS_USER_SEMAPHORE
// #define ipconfigSOCKET_HAS_USER_WAKE_CALLBACK
// #define ipconfigSUPPORT_SELECT_FUNCTION
// #define ipconfigSUPPORT_SIGNALS
// #define ipconfigUSE_CALLBACKS

// // Constants Affecting the ARP Behaviour
// #define ipconfigARP_CACHE_ENTRIES
// #define ipconfigARP_STORES_REMOTE_ADDRESSES
// #define ipconfigARP_USE_CLASH_DETECTION
// #define ipconfigMAX_ARP_AGE
// #define ipconfigMAX_ARP_RETRANSMISSIONS
// #define ipconfigUSE_ARP_REMOVE_ENTRY 
// #define ipconfigUSE_ARP_REVERSED_LOOKUP

// // Constants Affecting DHCP and Name Service Behaviour
// #define ipconfigDHCP_FALL_BACK_AUTO_IP
// #define ipconfigDHCP_REGISTER_HOSTNAME
// #define ipconfigDNS_CACHE_ADDRESSES_PER_ENTRY
// #define ipconfigDNS_CACHE_ENTRIES
// #define ipconfigDNS_CACHE_NAME_LENGTH
// #define ipconfigDNS_REQUEST_ATTEMPTS
// #define ipconfigDNS_USE_CALLBACKS
// #define ipconfigMAXIMUM_DISCOVER_TX_PERIOD
// #define ipconfigUSE_DHCP
// #define ipconfigUSE_DHCPv6
// #define ipconfigUSE_DHCP_HOOK
// #define ipconfigUSE_DNS
// #define ipconfigUSE_DNS_CACHE
// #define ipconfigUSE_LLMNR
// #define ipconfigUSE_NBNS
// #define ipconfigUSE_MDNS

// // Constants Affecting IP and ICMP Behaviour
// #define ipconfigUSE_IPv4
// #define ipconfigUSE_IPv6
// #define ipconfigFORCE_IP_DONT_FRAGMENT
// #define ipconfigICMP_TIME_TO_LIVE
// #define ipconfigIP_PASS_PACKETS_WITH_IP_OPTIONS
// #define ipconfigREPLY_TO_INCOMING_PINGS
// #define ipconfigSUPPORT_OUTGOING_PINGS

// // Constants Affecting ND Behaviour
// #define ipconfigND_CACHE_ENTRIES

// // Constants Affecting RA Behaviour
// #define ipconfigUSE_RA
// #define ipconfigRA_SEARCH_COUNT
// #define ipconfigRA_IP_TEST_COUNT

// // Constants Providing Target Support
// #define ipconfigHAS_INLINE_FUNCTIONS
// #define ipconfigRAND32
// #define ipconfigIS_VALID_PROG_ADDRESS( x )
// #define ipconfigPORT_SUPPRESS_WARNING



#endif