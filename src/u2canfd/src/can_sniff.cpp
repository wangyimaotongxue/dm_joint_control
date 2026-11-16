#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstddef>
#include <thread>
#include <unistd.h>

#include "protocol/pub_user.h"

std::atomic<bool> running(true);

void on_signal(int)
{
    running = false;
}

void rec_cb(usb_rx_frame_t* f)
{
    if (!f) return;
    printf("RX ch=%u id=0x%X canfd=%u brs=%u dlc=%u data:",
           f->head.channel, f->head.can_id, f->head.canfd, f->head.brs, f->head.dlc);
    for (int i = 0; i < 8; ++i) printf(" %02X", f->payload[i]);
    printf("\n");
    fflush(stdout);
}

void sent_cb(usb_rx_frame_t* f)
{
    if (!f) return;
    printf("TX ch=%u id=0x%X canfd=%u brs=%u dlc=%u\n",
           f->head.channel, f->head.can_id, f->head.canfd, f->head.brs, f->head.dlc);
    fflush(stdout);
}

void err_cb(usb_rx_frame_t* f)
{
    if (!f) return;
    printf("ERR ch=%u id=0x%X canfd=%u brs=%u dlc=%u\n",
           f->head.channel, f->head.can_id, f->head.canfd, f->head.brs, f->head.dlc);
    fflush(stdout);
}

void print_baud(device_handle* dev, const char* tag)
{
    device_baud_t baud = {0};
    if (device_channel_get_baudrate(dev, 0, &baud))
    {
        printf("baud[%s]: can=%d canfd=%d sp=%f fdsp=%f\n",
               tag, baud.can_baudrate, baud.canfd_baudrate, baud.can_sp, baud.canfd_sp);
    }
    else
    {
        printf("baud[%s]: get failed\n", tag);
    }
}

int main()
{
    std::signal(SIGINT, on_signal);

    damiao_handle* h = damiao_handle_create(DEV_USB2CANFD);
    damiao_print_version(h);

    int dev_cnt = damiao_handle_find_devices(h);
    printf("devices=%d\n", dev_cnt);

    device_handle* devs[16] = {0};
    int cnt = 0;
    damiao_handle_get_devices(h, devs, &cnt);
    printf("handles=%d\n", cnt);
    if (cnt == 0) return 1;

    device_handle* dev = devs[0];
    if (!device_open(dev))
    {
        printf("device_open failed\n");
        return 1;
    }
    printf("device_open ok\n");

    device_hook_to_rec(dev, rec_cb);
    device_hook_to_sent(dev, sent_cb);
    device_hook_to_err(dev, err_cb);

    // 库实际的顺序：先设波特率，再开通道
    bool b = device_channel_set_baud_with_sp(dev, 0, true, 1000000, 5000000, 0.75f, 0.75f);
    printf("set_baud(before open)=%d\n", b ? 1 : 0);
    print_baud(dev, "before open");

    bool o = device_open_channel(dev, 0);
    printf("open_channel=%d\n", o ? 1 : 0);

    if (o)
    {
        print_baud(dev, "after open");
        uint8_t read_cmd[8] = {0x01, 0x00, 0x33, 0x0A, 0x00, 0x00, 0x00, 0x00};
        while (running)
        {
                device_channel_send_fast(dev, 0, 0x7FF, 1, false, true, true, 8, read_cmd);
                printf("SEND CANFD\n");
            fflush(stdout);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    device_close_channel(dev, 0);
    device_close(dev);
    damiao_handle_destroy(h);
    return 0;
}
