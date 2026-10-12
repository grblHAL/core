#include <stdio.h>
#include "../../nvs_buffer.c"

grbl_hal_t hal;
static uint8_t ram[NVS_SIZE], persisted[NVS_SIZE];
static unsigned calls, fail_mask;

void report_message (const char *message, message_type_t type)
{
    (void)message;
    (void)type;
}

static bool write_nvs (uint32_t destination, uint8_t *source, uint32_t size, bool checksum)
{
    unsigned idx = (destination - NVS_ADDR_STARTUP_BLOCK) / (sizeof(stored_line_t) + NVS_CRC_BYTES);
    assert(idx < N_STARTUP_LINE && !checksum);
    assert(destination + size <= sizeof(persisted));
    calls++;
    if(fail_mask & bit(idx))
        return false;
    memcpy(persisted + destination, source, size);
    return NVS_TransferResult_OK;
}

static unsigned count_bits (unsigned value)
{
    unsigned count = 0;
    while(value) {
        count += value & 1;
        value >>= 1;
    }
    return count;
}

static bool check_retry (unsigned pending, unsigned failures)
{
    memset(ram, 0x23, sizeof(ram));
    memset(persisted, 0xFF, sizeof(persisted));
    memset(&settings_dirty, 0, sizeof(settings_dirty));
    settings_dirty.startup_lines = pending;
    settings_dirty.is_dirty = pending != 0;
    calls = 0;
    fail_mask = failures;
    unsigned remaining = pending & failures;

    nvs_buffer_sync_physical();
    if(calls != count_bits(pending) || settings_dirty.startup_lines != remaining || settings_dirty.is_dirty != (remaining != 0))
        return false;

    // Retry only failed entries; successful entries must not be written again.
    fail_mask = 0;
    nvs_buffer_sync_physical();
    if(calls != count_bits(pending) + count_bits(remaining) || settings_dirty.startup_lines || settings_dirty.is_dirty)
        return false;

    for(unsigned idx = 0; idx < N_STARTUP_LINE; idx++) {
        uint32_t offset = STARTLINE_ADDR(idx);
        if(pending & bit(idx)) {
            if(memcmp(persisted + offset, ram + offset, sizeof(stored_line_t) + NVS_CRC_BYTES))
                return false;
        } else if(persisted[offset] != 0xFF)
            return false;
    }

    unsigned previous_calls = calls;
    nvs_buffer_sync_physical();
    return calls == previous_calls;
}

int main (void)
{
    nvsbuffer.addr = ram;
    physical_nvs.memcpy_to_nvs = write_nvs;
    unsigned errors = 0, cases = 0;
    for(unsigned pending = 0; pending < bit(N_STARTUP_LINE); pending++) {
        for(unsigned failures = 0; failures < bit(N_STARTUP_LINE); failures++) {
            cases++;
            if(!check_retry(pending, failures)) {
                fprintf(stderr, "Failed startup retry: pending=%u failures=%u\n", pending, failures);
                errors++;
            }
        }
    }
    printf("%u startup NVS scenarios, %u failed\n", cases, errors);
    return errors != 0;
}
