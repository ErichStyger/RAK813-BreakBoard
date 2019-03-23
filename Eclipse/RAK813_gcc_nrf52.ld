/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
 /* FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x80000 */
 /* RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x8000 */
/*  FLASH (rx) : ORIGIN = 0x1c000, LENGTH = 0x64000 */
  FLASH (rx) : ORIGIN = 0x1c000+0x7000, LENGTH = 0x64000-0x7000 
 /* RAM (rwx) :  ORIGIN = 0x200013c8, LENGTH = 0x6c38 */
 RAM (rwx) :  ORIGIN = 0x20002a68, LENGTH = 0xd598
}

SECTIONS
{
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

SECTIONS
{

.log_dynamic_data :
{
PROVIDE(__start_log_dynamic_data = .);
KEEP(*(.log_dynamic_data))
PROVIDE(__stop_log_dynamic_data = .);
} > FLASH


.log_const_data :
{
PROVIDE(__start_log_const_data = .);
KEEP(*(.log_const_data))
PROVIDE(__stop_log_const_data = .);
} > FLASH

.sdh_soc_observers :
{
PROVIDE(__start_sdh_soc_observers = .);
KEEP(*(SORT(.sdh_soc_observers*)))
PROVIDE(__stop_sdh_soc_observers = .);
} > FLASH


.sdh_stack_observers :
{
PROVIDE(__start_sdh_stack_observers = .);
KEEP(*(SORT(.sdh_stack_observers*)))
PROVIDE(__stop_sdh_stack_observers = .);
} > FLASH

.sdh_req_observers :
{
PROVIDE(__start_sdh_req_observers = .);
KEEP(*(SORT(.sdh_req_observers*)))
PROVIDE(__stop_sdh_req_observers = .);
} > FLASH

.sdh_state_observers :
{
PROVIDE(__start_sdh_state_observers = .);
KEEP(*(SORT(.sdh_state_observers*)))
PROVIDE(__stop_sdh_state_observers = .);
} > FLASH

.sdh_ble_observers :
{
PROVIDE(__start_sdh_ble_observers = .);
KEEP(*(SORT(.sdh_ble_observers*)))
PROVIDE(__stop_sdh_ble_observers = .);
} > FLASH

} INSERT AFTER .text

INCLUDE "nrf5x_common.ld"

