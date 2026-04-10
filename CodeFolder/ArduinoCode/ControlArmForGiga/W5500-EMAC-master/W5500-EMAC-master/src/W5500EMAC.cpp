
#include "W5500EMAC.h"
#include <Arduino.h>
#include <SPI.h>

#include <EthernetInterface.h>
#include <mbed_events.h>

#ifndef W5500_CS
#ifdef ARDUINO_ARCH_MBED_PORTENTA
#define W5500_CS D5  // MKR ETH shield CS
#else
#define W5500_CS PIN_SPI_SS
#endif
#endif

#define W5500_BUFF_ALIGNMENT              4U
#define W5500_ETH_MTU_SIZE               1500U
#define W5500_ETH_IF_NAME                "W5500"

using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;

#define W5500_RECEIVE_TASK_PERIOD_MS      20ms
#define W5500_LINK_STATUS_TASK_PERIOD_MS 500ms

#define SPI_ETHERNET_SETTINGS SPISettings(20000000, MSBFIRST, SPI_MODE0)

W5500EMAC::W5500EMAC(SPIClass &_spi) : spi(_spi), driver(W5500_CS, _spi) {
}

W5500EMAC& W5500EMAC::get_instance(void) {
  static W5500EMAC emac(SPI1);
  return emac;
}

/**
 * Return maximum transmission unit
 *
 * @return     MTU in bytes
 */
uint32_t W5500EMAC::get_mtu_size(void) const {
  return W5500_ETH_MTU_SIZE;
}

/**
 * Gets memory buffer alignment preference
 *
 * Gets preferred memory buffer alignment of the Emac device. IP stack may
 * or may not align link out memory buffer chains using the alignment.
 *
 * @return         Memory alignment requirement in bytes
 */
uint32_t W5500EMAC::get_align_preference(void) const {
  return W5500_BUFF_ALIGNMENT;
}

/**
 * Return interface name
 *
 * @param name Pointer to where the name should be written
 * @param size Maximum number of character to copy
 */
void W5500EMAC::get_ifname(char *name, uint8_t size) const {
  memcpy(name, W5500_ETH_IF_NAME, (size < sizeof(W5500_ETH_IF_NAME)) ? size : sizeof(W5500_ETH_IF_NAME));
}

/**
 * Returns size of the underlying interface HW address size.
 *
 * @return     HW address size in bytes
 */
uint8_t W5500EMAC::get_hwaddr_size(void) const {
  return W5500_HWADDR_SIZE;
}

/**
 * Return interface-supplied HW address
 *
 * Copies HW address to provided memory, @param addr has to be of correct
 * size see @a get_hwaddr_size
 *
 * HW address need not be provided if this interface does not have its own
 * HW address configuration; stack will choose address from central system
 * configuration if the function returns false and does not write to addr.
 *
 * @param addr HW address for underlying interface
 * @return     true if HW address is available
 */
bool W5500EMAC::get_hwaddr(uint8_t *addr) const {
  memcpy(addr, macAddr, W5500_HWADDR_SIZE);
  return true;
}

/**
 * Set HW address for interface
 *
 * Provided address has to be of correct size, see @a get_hwaddr_size
 *
 * Called to set the MAC address to actually use - if @a get_hwaddr is
 * provided the stack would normally use that, but it could be overridden,
 * eg for test purposes.
 *
 * @param addr Address to be set
 */
void W5500EMAC::set_hwaddr(const uint8_t *addr) {
  memcpy(macAddr, addr, W5500_HWADDR_SIZE);
}

/**
 * Initializes the HW
 *
 * @return True on success, False in case of an error.
 */
bool W5500EMAC::power_up(void) {

  spi.begin();
  spi.beginTransaction(SPI_ETHERNET_SETTINGS);
  bool ret = driver.begin(macAddr);
  spi.endTransaction();
  if (!ret)
    return false;
  receiveTaskHandle = mbed::mbed_event_queue()->call_every(W5500_RECEIVE_TASK_PERIOD_MS, mbed::callback(this, &W5500EMAC::receiveTask));
  linkStatusTaskHandle = mbed::mbed_event_queue()->call_every(W5500_LINK_STATUS_TASK_PERIOD_MS, mbed::callback(this, &W5500EMAC::linkStatusTask));
  return true;
}

/**
 * Deinitializes the HW
 *
 */
void W5500EMAC::power_down(void) {
  mbed::mbed_event_queue()->cancel(receiveTaskHandle);
  mbed::mbed_event_queue()->cancel(linkStatusTaskHandle);
  driver.end();
}

/**
 * Sends the packet over the link
 *
 * That can not be called from an interrupt context.
 *
 * @param buf  Packet to be send
 * @return     True if the packet was send successfully, False otherwise
 */
bool W5500EMAC::link_out(emac_mem_buf_t *buf) {

  if (buf == NULL)
    return false;

  // If buffer is chained or not aligned then make a contiguous aligned copy of it
  if (memoryManager->get_next(buf) || reinterpret_cast<uint32_t>(memoryManager->get_ptr(buf)) % W5500_BUFF_ALIGNMENT) {
    emac_mem_buf_t *copy_buf;
    copy_buf = memoryManager->alloc_heap(memoryManager->get_total_len(buf), W5500_BUFF_ALIGNMENT);
    if (NULL == copy_buf) {
      memoryManager->free(buf);
      return false;
    }

    // Copy to new buffer and free original
    memoryManager->copy(copy_buf, buf);
    memoryManager->free(buf);
    buf = copy_buf;
  }

  uint16_t len = memoryManager->get_len(buf);
  uint8_t *data = (uint8_t*) (memoryManager->get_ptr(buf));
  ethLockMutex.lock();
  spi.beginTransaction(SPI_ETHERNET_SETTINGS);
  bool ret = driver.sendFrame(data, len) == len;
  spi.endTransaction();
  memoryManager->free(buf);
  ethLockMutex.unlock();

  return ret;
}

void W5500EMAC::linkStatusTask() {
  ethLockMutex.lock();
  spi.beginTransaction(SPI_ETHERNET_SETTINGS);
  static bool linked = !driver.isLinked();
  if (linked != driver.isLinked()) {
    linked = driver.isLinked();
    emac_link_state_cb(linked);
  }
  spi.endTransaction();
  ethLockMutex.unlock();
}

void W5500EMAC::receiveTask() {
  if (!emac_link_input_cb)
    return;
  ethLockMutex.lock();
  spi.beginTransaction(SPI_ETHERNET_SETTINGS);
  emac_mem_buf_t *buf = driver.readFrame(memoryManager);
  spi.endTransaction();
  if (buf != NULL) {
    emac_link_input_cb(buf);
  }
  ethLockMutex.unlock();
}

/**
 * Sets a callback that needs to be called for packets received for that
 * interface
 *
 * @param input_cb Function to be register as a callback
 */
void W5500EMAC::set_link_input_cb(emac_link_input_cb_t input_cb) {
  emac_link_input_cb = input_cb;
}

/**
 * Sets a callback that needs to be called on link status changes for given
 * interface
 *
 * @param state_cb Function to be register as a callback
 */
void W5500EMAC::set_link_state_cb(emac_link_state_change_cb_t state_cb) {
  emac_link_state_cb = state_cb;
}

/** Add device to a multicast group
 *
 * @param address  A multicast group hardware address
 */
void W5500EMAC::add_multicast_group(const uint8_t *address) {

}

/** Remove device from a multicast group
 *
 * @param address  A multicast group hardware address
 */
void W5500EMAC::remove_multicast_group(const uint8_t *address) {

}

/** Request reception of all multicast packets
 *
 * @param all True to receive all multicasts
 *            False to receive only multicasts addressed to specified groups
 */
void W5500EMAC::set_all_multicast(bool all) {

}

/** Sets memory manager that is used to handle memory buffers
 *
 * @param mem_mngr Pointer to memory manager
 */
void W5500EMAC::set_memory_manager(EMACMemoryManager &mem_mngr) {
  memoryManager = &mem_mngr;
}

EMAC& EMAC::get_default_instance() {
  return W5500EMAC::get_instance();
}

EthInterface *EthInterface::get_target_default_instance()
{
  static EthernetInterface ethernet;
  return &ethernet;
}
