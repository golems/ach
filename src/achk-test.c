#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "../ach.h"
#include <errno.h>
#include <sys/select.h>

void usage(void)
{
  printf ("achk-test <option>\n");
  printf ("  -c <name>          - create channel\n");
  printf ("  -r <name>          - remove channel\n");
  printf ("  -p <ch> <data>     - put data into channel\n");
  printf ("  -g <ch> new|old    - get data from channel\n");
  printf ("  -ga <ch>           - get all unread from channel\n");
  printf ("  -gb <ch>           - get with blocking - wait for more \n");
  printf ("  -s <ch1> <ch2>     - select on ch1 and ch2 - read 5 messages\n");
}

#define ACHCTRL "/dev/achctrl"

int createdevice(const char* name)
{
  int ret;
  int fd = open(ACHCTRL, O_WRONLY|O_APPEND);
  if (fd < 0) {
    printf("Error opening %s\n", ACHCTRL);
    return -1;
  }

  struct ach_ctrl_create_ch arg;
  arg.frame_cnt = 8;
  arg.frame_size=128;
  strcpy(arg.name, name);

  ret = ioctl(fd, ACH_CTRL_CREATE_CH, &arg);
  if (ret < 0)
    printf("Error creating ach device: %s\n", name);
  else
    printf("ach device %s created\n", name);

  close (fd);
  return ret;

}

int removedevice(const char* name)
{
  int ret;
  int fd = open(ACHCTRL, O_WRONLY|O_APPEND);

  if (fd < 0) {
    printf("Error opening %s\n", ACHCTRL);
    return -1;
  }

  struct ach_ctrl_unlink_ch arg;
  strcpy(arg.name, name);

  ret = ioctl(fd, ACH_CTRL_UNLINK_CH, &arg);
  if (ret < 0)
    printf("Error removing ach device %s\n", name);
  else
    printf("ach device %s removed\n", name);

  close (fd);
  return ret;
}

int putdata (const char* channel, const char* data)
{
  int fd = open(channel, O_WRONLY|O_APPEND);
  if (fd < 0) {
    printf ("Error opening %s\n", channel);
    return -1;
  }

  ssize_t len = write(fd, data, strlen(data));
  if (len >= 0)
    printf("Wrote %zu bytes to channel %s\n", len, channel);
  else
    printf("Error writing to channel %s\n", channel);

  close(fd);
  return 0;
}

int readdata(int fd, unsigned int flag)
{
  int res = 0;

  printf ("In readdata: fd=%d, flag=0x%02x\n", fd, flag);
  struct achk_opt mode;
  mode.mode = flag;
  if (0 > ioctl(fd, ACH_CH_SET_MODE, &flag)) {
    printf("Failed setting mode\n");
    res = -1;
    goto out;
  }

  struct ach_ch_status stat;
  if (0 > ioctl(fd, ACH_CH_GET_STATUS, &stat)) {
    printf("Failed getting status\n");
    res = -1;
    goto out;
  }
  /*
  if (! stat.new) {
    res = -1;
    printf("No unread messages in channel\n");
    goto out;
  }
  */
  char buf[200+1];
  int len;
  printf("Reading using fd=%d\n", fd);
  len = read(fd, buf, 200);
  if (0 > len) {
    printf("Failed reading from channel: errno: %d\n", errno);
    res = -1;
    goto out;
  }
  buf[len] = '\0';
  printf("Data read (%d bytes): %s\n", len, buf);

 out:
  return res;
}

int getwithwait(const char* channel)
{
  int res = 0;
  printf ("Getting all unread data\n");
  int fd = open(channel, O_RDWR);
  if (fd < 0) {
    printf ("Error opening %s\n", channel);
    return -1;
  }
  unsigned int flag = ACH_CH_MODE_LAST | ACH_CH_MODE_WAIT;
  while (!res) {
    res = readdata(fd, flag);
  }

  close(fd);
  return 0;
}

int getalldata(const char* channel)
{
  int res = 0;
  printf ("Getting all unread data\n");
  int fd = open(channel, O_RDWR);
  if (fd < 0) {
    printf ("Error opening %s\n", channel);
    return -1;
  }
  unsigned int flag = 0; /* Get old */
  while (!res) {
    res = readdata(fd, flag);
  }

  close(fd);
  return 0;
}

int getolddata(const char* channel)
{
  int res = 0;
  printf ("Getting next data\n");
  int fd = open(channel, O_RDWR);
  if (fd < 0) {
    printf ("Error opening %s\n", channel);
    return -1;
  }

  unsigned int flag = 0;

  res = readdata(fd, flag);

 out:
  close(fd);
  return res;

}

int getnewdata(const char* channel)
{
  int res = 0;
  printf("Getting newest data\n");
  int fd = open(channel, O_RDWR);
  if (fd < 0) {
    printf ("Error opening %s\n", channel);
    return -1;
  }

  unsigned int flag = ACH_CH_MODE_LAST;

  res = readdata(fd, flag);

 out:
  close(fd);
  return res;
}

int selecttest(const char* ch1, const char* ch2)
{
  int fd1 = open(ch1, O_RDWR);
  if (fd1 < 0) {
    printf("Error opening %s\n", ch1);
    return -1;
  }
  int fd2 = open(ch2, O_RDWR);
  if (fd2 < 0) {
    printf("Error opening %s\n", ch2);
    close(fd1);
    return -1;
  }

  int cnt = 5;
  while (cnt) {
    fd_set readset;
    FD_ZERO(&readset);
    FD_SET(fd1, &readset);
    FD_SET(fd2, &readset);
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    int res = select(fd2+1, &readset, NULL, NULL, &tv);
    if (res == 0) {
      printf("select timed out\n");
    }
    if (res > 0) {
      cnt--;
      if (FD_ISSET(fd1, &readset)) {
        readdata(fd1, 0);
      }
      if (FD_ISSET(fd2, &readset)) {
        readdata(fd2, 0);
      }
    }
  }
 out:
  close(fd1);
  close(fd2);

  return 0;
}

int main(int argc, char ** argv)
{
  printf ("In main of %s: argc=%d\n",argv[0], argc);

  if (argc < 3) {
    usage();
    exit (1);
  }

  if (0 == strcmp(argv[1],"-c")) {
    createdevice(argv[2]);

  }
  else if (0 == strcmp(argv[1],"-r")) {
    removedevice(argv[2]);
  }
  else if (0 == strcmp(argv[1], "-p")) {
    if (argc < 4)
      goto usage;

    putdata(argv[2], argv[3]);
  }
  else if (0 == strcmp(argv[1], "-g")) {
    if (argc < 4)
        goto usage;

    if (0 == strcmp(argv[3], "new")) {
      getnewdata(argv[2]);

    } else {
      getolddata(argv[2]);

    }
  } else if (0 == strcmp(argv[1], "-ga")) {
    getalldata(argv[2]);
  } else if (0 == strcmp(argv[1], "-gb")) {
    getwithwait(argv[2]);
  } else if (0 == strcmp(argv[1], "-s")) {
    if (argc < 4)
      goto usage;
    selecttest(argv[2], argv[3]);
  } else {
    goto usage;
  }

  return 0;

 usage:
      usage();
      exit(1);

      return 1;

}
