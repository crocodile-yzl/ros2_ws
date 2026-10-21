// SPDX-License-Identifier: Apache-2.0
/**
 * d435i_video_c —— Intel RealSense D435i 图像发布节点，纯 C 实现。
 *
 * 采集用 librealsense2 的 C API（rs2_*），发布用 ROS 2 的 C 客户端库 rcl +
 * rosidl 生成的 C 消息类型（sensor_msgs__msg__Image）。不依赖 rclcpp/rclpy。
 *
 * 话题：
 *   <topic>         sensor_msgs/Image  bgr8   彩色图（默认 /video）
 *   <depth-topic>   sensor_msgs/Image  16UC1  深度图，毫米（默认 /video/depth）
 *
 * 用法示例：
 *   d435i_video_c
 *   d435i_video_c --depth --align
 *   d435i_video_c --width 640 --height 480 --fps 30 --depth --align
 *   d435i_video_c --serial 123622270123 --topic /video2
 */

#define _POSIX_C_SOURCE 200809L

#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include <librealsense2/rs.h>
#include <librealsense2/h/rs_config.h>
#include <librealsense2/h/rs_context.h>
#include <librealsense2/h/rs_frame.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_processing.h>
#include <librealsense2/h/rs_types.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/image.h>

#define LOG_PREFIX "[d435i_video_c] "
#define MAX_FRAMES_PER_SET 4
#define STATS_EVERY_N 150

static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig)
{
  (void)sig;
  g_running = 0;
}

/* rcl 的清理函数带 warn_unused_result，显式消费掉返回值 */
static void rcl_ignore(rcl_ret_t rc)
{
  (void)rc;
}

/* ------------------------------------------------------------------ */
/* 参数                                                               */
/* ------------------------------------------------------------------ */

typedef struct
{
  int width;
  int height;
  int fps;
  bool enable_depth;
  int depth_width;
  int depth_height;
  int depth_fps;
  bool align_depth;
  bool reliable;
  char serial[64];
  char topic[128];
  char depth_topic[128];
  char frame_id[128];
  int timeout_ms;
} options_t;

static void options_default(options_t *o)
{
  memset(o, 0, sizeof(*o));
  o->width = 1280;
  o->height = 720;
  o->fps = 30;
  o->enable_depth = false;
  o->depth_width = 640;
  o->depth_height = 480;
  o->depth_fps = 30;
  o->align_depth = false;
  o->reliable = false;
  o->serial[0] = '\0';
  snprintf(o->topic, sizeof(o->topic), "/video");
  snprintf(o->depth_topic, sizeof(o->depth_topic), "/video/depth");
  snprintf(o->frame_id, sizeof(o->frame_id), "camera_link");
  o->timeout_ms = 5000;
}

static void print_usage(const char *prog)
{
  printf(
      "用法: %s [选项]\n"
      "\n"
      "彩色流:\n"
      "  --width N            宽度，默认 1280\n"
      "  --height N           高度，默认 720\n"
      "  --fps N              帧率，默认 30\n"
      "  --topic NAME         彩色话题，默认 /video\n"
      "\n"
      "深度流:\n"
      "  --depth              开启深度流\n"
      "  --depth-width N      深度宽度，默认 640\n"
      "  --depth-height N     深度高度，默认 480\n"
      "  --depth-fps N        深度帧率，默认 30\n"
      "  --align              把深度对齐到彩色\n"
      "  --depth-topic NAME   深度话题，默认 /video/depth\n"
      "\n"
      "其它:\n"
      "  --serial SN          按序列号选择相机（多机时）\n"
      "  --frame-id ID        header.frame_id，默认 camera_link\n"
      "  --reliable           使用 RELIABLE QoS（默认 BEST_EFFORT）\n"
      "  -h, --help           显示帮助\n",
      prog);
}

static bool parse_bool(const char *s)
{
  if (s == NULL || s[0] == '\0')
  {
    return true;
  }
  return strcmp(s, "1") == 0 || strcasecmp(s, "true") == 0 ||
         strcasecmp(s, "yes") == 0 || strcasecmp(s, "on") == 0;
}

static void parse_args(int argc, char **argv, options_t *o)
{
  /* launch 可能追加 --ros-args ...，这些由 rcl 处理，这里直接截断 */
  for (int i = 1; i < argc; ++i)
  {
    if (strcmp(argv[i], "--ros-args") == 0)
    {
      argc = i;
      break;
    }
  }

  enum
  {
    OPT_DEPTH_WIDTH = 1000,
    OPT_DEPTH_HEIGHT,
    OPT_DEPTH_FPS
  };

  static const struct option long_opts[] = {
      {"width", required_argument, NULL, 'w'},
      {"height", required_argument, NULL, 'e'},
      {"fps", required_argument, NULL, 'f'},
      {"depth", optional_argument, NULL, 'd'},
      {"align", optional_argument, NULL, 'a'},
      {"depth-width", required_argument, NULL, OPT_DEPTH_WIDTH},
      {"depth-height", required_argument, NULL, OPT_DEPTH_HEIGHT},
      {"depth-fps", required_argument, NULL, OPT_DEPTH_FPS},
      {"serial", optional_argument, NULL, 's'},
      {"topic", required_argument, NULL, 't'},
      {"depth-topic", required_argument, NULL, 'T'},
      {"frame-id", required_argument, NULL, 'F'},
      {"reliable", optional_argument, NULL, 'R'},
      {"help", no_argument, NULL, 'h'},
      {NULL, 0, NULL, 0}};

  int c;
  while ((c = getopt_long(argc, argv, "w:e:f:das:t:T:F:Rh", long_opts, NULL)) != -1)
  {
    switch (c)
    {
    case 'w':
      o->width = atoi(optarg);
      break;
    case 'e':
      o->height = atoi(optarg);
      break;
    case 'f':
      o->fps = atoi(optarg);
      break;
    case 'd':
      o->enable_depth = parse_bool(optarg);
      break;
    case 'a':
      o->align_depth = parse_bool(optarg);
      break;
    case OPT_DEPTH_WIDTH:
      o->depth_width = atoi(optarg);
      break;
    case OPT_DEPTH_HEIGHT:
      o->depth_height = atoi(optarg);
      break;
    case OPT_DEPTH_FPS:
      o->depth_fps = atoi(optarg);
      break;
    case 's':
      /* 空值表示自动选择第一台，避免把后续选项误当成序列号 */
      if (optarg != NULL && optarg[0] != '\0')
      {
        snprintf(o->serial, sizeof(o->serial), "%s", optarg);
      }
      else
      {
        o->serial[0] = '\0';
      }
      break;
    case 't':
      snprintf(o->topic, sizeof(o->topic), "%s", optarg);
      break;
    case 'T':
      snprintf(o->depth_topic, sizeof(o->depth_topic), "%s", optarg);
      break;
    case 'F':
      snprintf(o->frame_id, sizeof(o->frame_id), "%s", optarg);
      break;
    case 'R':
      o->reliable = parse_bool(optarg);
      break;
    case 'h':
      print_usage(argv[0]);
      exit(0);
    default:
      print_usage(argv[0]);
      exit(2);
    }
  }
}

/* ------------------------------------------------------------------ */
/* 图像发布器（复用一块 buffer，避免每帧 malloc）                      */
/* ------------------------------------------------------------------ */

typedef struct
{
  rcl_publisher_t publisher;
  sensor_msgs__msg__Image msg;
  size_t capacity;
} image_pub_t;

static bool image_pub_init(image_pub_t *p, rcl_node_t *node, const char *topic,
                           const char *frame_id, const char *encoding, bool reliable)
{
  memset(p, 0, sizeof(*p));

  if (!sensor_msgs__msg__Image__init(&p->msg))
  {
    fprintf(stderr, LOG_PREFIX "初始化消息失败\n");
    return false;
  }
  if (!rosidl_runtime_c__String__assign(&p->msg.header.frame_id, frame_id))
  {
    fprintf(stderr, LOG_PREFIX "分配 frame_id 失败\n");
    return false;
  }
  if (!rosidl_runtime_c__String__assign(&p->msg.encoding, encoding))
  {
    fprintf(stderr, LOG_PREFIX "分配 encoding 失败\n");
    return false;
  }
  p->msg.is_bigendian = 0;

  p->publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  pub_opts.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  pub_opts.qos.depth = 1;
  pub_opts.qos.reliability = reliable ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                                      : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

  rcl_ret_t rc = rcl_publisher_init(
      &p->publisher, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image), topic, &pub_opts);
  if (rc != RCL_RET_OK)
  {
    fprintf(stderr, LOG_PREFIX "创建话题 %s 失败: %s\n", topic, rcl_get_error_string().str);
    rcl_reset_error();
    return false;
  }

  return true;
}

static bool image_pub_publish(image_pub_t *p, const void *data, size_t bytes,
                              uint32_t width, uint32_t height, uint32_t step)
{
  if (bytes > p->capacity)
  {
    uint8_t *buf = (uint8_t *)realloc(p->msg.data.data, bytes);
    if (buf == NULL)
    {
      fprintf(stderr, LOG_PREFIX "内存不足，需要 %zu 字节\n", bytes);
      return false;
    }
    p->msg.data.data = buf;
    p->capacity = bytes;
  }

  memcpy(p->msg.data.data, data, bytes);
  p->msg.data.size = bytes;
  p->msg.data.capacity = p->capacity;
  p->msg.width = width;
  p->msg.height = height;
  p->msg.step = step;

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  p->msg.header.stamp.sec = (int32_t)ts.tv_sec;
  p->msg.header.stamp.nanosec = (uint32_t)ts.tv_nsec;

  rcl_ret_t rc = rcl_publish(&p->publisher, &p->msg, NULL);
  if (rc != RCL_RET_OK)
  {
    fprintf(stderr, LOG_PREFIX "发布失败: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    return false;
  }
  return true;
}

static void image_pub_fini(image_pub_t *p, rcl_node_t *node)
{
  if (rcl_publisher_is_valid(&p->publisher))
  {
    rcl_ignore(rcl_publisher_fini(&p->publisher, node));
  }
  sensor_msgs__msg__Image__fini(&p->msg);
  p->capacity = 0;
}

/* ------------------------------------------------------------------ */
/* librealsense 辅助                                                   */
/* ------------------------------------------------------------------ */

static bool rs_ok(rs2_error **e, const char *what)
{
  if (*e == NULL)
  {
    return true;
  }
  fprintf(stderr, LOG_PREFIX "%s 失败: %s\n", what, rs2_get_error_message(*e));
  rs2_free_error(*e);
  *e = NULL;
  return false;
}

/* ------------------------------------------------------------------ */
/* 主流程                                                             */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
  options_t opt;
  options_default(&opt);
  parse_args(argc, argv, &opt);

  int exit_code = 1;   /* 走到 cleanup 的错误路径都算失败 */

  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = on_signal;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);

  rcl_ret_t rc;
  rcl_allocator_t alloc = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rc = rcl_init_options_init(&init_options, alloc);
  if (rc != RCL_RET_OK)
  {
    fprintf(stderr, LOG_PREFIX "rcl_init_options_init 失败\n");
    return 1;
  }

  rcl_context_t context = rcl_get_zero_initialized_context();
  rc = rcl_init(0, NULL, &init_options, &context);
  if (rc != RCL_RET_OK)
  {
    fprintf(stderr, LOG_PREFIX "rcl_init 失败: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    rcl_ignore(rcl_init_options_fini(&init_options));
    return 1;
  }

  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_options = rcl_node_get_default_options();
  rc = rcl_node_init(&node, "d435i_video_c", "/", &context, &node_options);
  if (rc != RCL_RET_OK)
  {
    fprintf(stderr, LOG_PREFIX "rcl_node_init 失败: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    rcl_ignore(rcl_shutdown(&context));
    rcl_ignore(rcl_context_fini(&context));
    rcl_ignore(rcl_init_options_fini(&init_options));
    return 1;
  }

  image_pub_t color_pub;
  image_pub_t depth_pub;
  bool color_ready = image_pub_init(&color_pub, &node, opt.topic, opt.frame_id,
                                    "bgr8", opt.reliable);
  bool depth_ready = false;
  if (!color_ready)
  {
    goto cleanup_node;
  }
  if (opt.enable_depth)
  {
    depth_ready = image_pub_init(&depth_pub, &node, opt.depth_topic,
                                 opt.frame_id, "16UC1", opt.reliable);
    if (!depth_ready)
    {
      goto cleanup_pubs;
    }
  }

  /* ---------------- RealSense ---------------- */
  rs2_error *e = NULL;
  rs2_context *rs_ctx = NULL;
  rs2_device_list *dev_list = NULL;
  rs2_pipeline *pipe = NULL;
  rs2_config *cfg = NULL;
  rs2_pipeline_profile *profile = NULL;
  rs2_processing_block *align_block = NULL;
  rs2_frame_queue *align_queue = NULL;

  rs_ctx = rs2_create_context(RS2_API_VERSION, &e);
  if (!rs_ok(&e, "创建 RealSense 上下文"))
  {
    goto cleanup_rs;
  }

  dev_list = rs2_query_devices(rs_ctx, &e);
  if (!rs_ok(&e, "枚举设备"))
  {
    goto cleanup_rs;
  }
  int device_count = rs2_get_device_count(dev_list, &e);
  if (!rs_ok(&e, "读取设备数量"))
  {
    goto cleanup_rs;
  }
  if (device_count <= 0)
  {
    fprintf(stderr,
            LOG_PREFIX "没有找到 RealSense 设备。检查：\n"
                       "  1) lsusb -d 8086:0b3a  是否能看到相机\n"
                       "  2) 容器是否映射了 /dev/bus/usb（见 compose 的 volumes）\n"
                       "  3) 是否插在 USB 3.0 口并使用原装线\n");
    goto cleanup_rs;
  }
  printf(LOG_PREFIX "发现 %d 个 RealSense 设备\n", device_count);

  pipe = rs2_create_pipeline(rs_ctx, &e);
  if (!rs_ok(&e, "创建 pipeline"))
  {
    goto cleanup_rs;
  }

  cfg = rs2_create_config(&e);
  if (!rs_ok(&e, "创建配置"))
  {
    goto cleanup_rs;
  }

  if (opt.serial[0] != '\0')
  {
    rs2_config_enable_device(cfg, opt.serial, &e);
    if (!rs_ok(&e, "按序列号选择设备"))
    {
      goto cleanup_rs;
    }
    printf(LOG_PREFIX "按序列号选择: %s\n", opt.serial);
  }

  rs2_config_enable_stream(cfg, RS2_STREAM_COLOR, 0, opt.width, opt.height,
                           RS2_FORMAT_BGR8, opt.fps, &e);
  if (!rs_ok(&e, "配置彩色流"))
  {
    fprintf(stderr, LOG_PREFIX "提示: 该组合可能不被支持，用 rs-enumerate-devices 查看\n");
    goto cleanup_rs;
  }

  if (opt.enable_depth)
  {
    rs2_config_enable_stream(cfg, RS2_STREAM_DEPTH, 0, opt.depth_width,
                             opt.depth_height, RS2_FORMAT_Z16, opt.depth_fps, &e);
    if (!rs_ok(&e, "配置深度流"))
    {
      goto cleanup_rs;
    }
  }

  profile = rs2_pipeline_start_with_config(pipe, cfg, &e);
  if (!rs_ok(&e, "启动管线"))
  {
    fprintf(stderr,
            LOG_PREFIX "提示: 常见原因是分辨率/帧率组合不支持，或 USB 带宽不足\n");
    goto cleanup_rs;
  }

  printf(LOG_PREFIX "彩色 %dx%d@%d bgr8", opt.width, opt.height, opt.fps);
  if (opt.enable_depth)
  {
    printf(" | 深度 %dx%d@%d z16%s", opt.depth_width, opt.depth_height,
           opt.depth_fps, opt.align_depth ? "（对齐到彩色）" : "");
  }
  printf("\n");
  printf(LOG_PREFIX "发布 %s (bgr8, %s)\n", opt.topic,
         opt.reliable ? "RELIABLE" : "BEST_EFFORT");
  if (opt.enable_depth)
  {
    printf(LOG_PREFIX "发布 %s (16UC1, %s)\n", opt.depth_topic,
           opt.reliable ? "RELIABLE" : "BEST_EFFORT");
  }
  fflush(stdout);

  if (opt.enable_depth && opt.align_depth)
  {
    align_block = rs2_create_align(RS2_STREAM_COLOR, &e);
    if (!rs_ok(&e, "创建对齐处理块"))
    {
      goto cleanup_rs;
    }
    align_queue = rs2_create_frame_queue(1, &e);
    if (!rs_ok(&e, "创建对齐队列"))
    {
      goto cleanup_rs;
    }
    rs2_start_processing_queue(align_block, align_queue, &e);
    if (!rs_ok(&e, "启动对齐队列"))
    {
      goto cleanup_rs;
    }
  }

  /* ---------------- 主循环 ---------------- */
  uint32_t published = 0;
  uint32_t in_window = 0;
  struct timespec win_start;
  clock_gettime(CLOCK_MONOTONIC, &win_start);

  while (g_running && rcl_context_is_valid(&context))
  {
    rs2_frame *frames = rs2_pipeline_wait_for_frames(pipe, (unsigned int)opt.timeout_ms, &e);
    if (!rs_ok(&e, "等待帧"))
    {
      continue;
    }

    rs2_frame *source = frames;
    rs2_frame *aligned = NULL;

    if (align_block != NULL)
    {
      rs2_process_frame(align_block, frames, &e);
      if (!rs_ok(&e, "深度对齐"))
      {
        rs2_release_frame(frames);
        continue;
      }
      aligned = rs2_wait_for_frame(align_queue, (unsigned int)opt.timeout_ms, &e);
      if (!rs_ok(&e, "获取对齐结果"))
      {
        rs2_release_frame(frames);
        continue;
      }
      source = aligned;
    }

    for (int i = 0; i < MAX_FRAMES_PER_SET; ++i)
    {
      rs2_error *fe = NULL;
      rs2_frame *frame = rs2_extract_frame(source, i, &fe);
      if (fe != NULL)
      {
        rs2_free_error(fe);
        break;
      }
      if (frame == NULL)
      {
        break;
      }

      const rs2_stream_profile *sp = rs2_get_frame_stream_profile(frame, &e);
      rs2_stream stream = RS2_STREAM_ANY;
      rs2_format format = RS2_FORMAT_ANY;
      int index = 0, unique_id = 0, framerate = 0;
      if (rs_ok(&e, "读取流信息"))
      {
        rs2_get_stream_profile_data(sp, &stream, &format, &index, &unique_id,
                                    &framerate, &e);
        if (!rs_ok(&e, "解析流信息"))
        {
          continue;
        }
      }
      else
      {
        continue;
      }

      const void *data = rs2_get_frame_data(frame, &e);
      int width = rs2_get_frame_width(frame, &e);
      int height = rs2_get_frame_height(frame, &e);
      int stride = rs2_get_frame_stride_in_bytes(frame, &e);
      if (!rs_ok(&e, "读取帧数据"))
      {
        continue;
      }
      if (data == NULL || width <= 0 || height <= 0 || stride <= 0)
      {
        continue;
      }

      size_t bytes = (size_t)stride * (size_t)height;

      if (stream == RS2_STREAM_COLOR && color_ready)
      {
        if (image_pub_publish(&color_pub, data, bytes, (uint32_t)width,
                              (uint32_t)height, (uint32_t)stride))
        {
          ++published;
          ++in_window;
        }
      }
      else if (stream == RS2_STREAM_DEPTH && depth_ready)
      {
        image_pub_publish(&depth_pub, data, bytes, (uint32_t)width,
                          (uint32_t)height, (uint32_t)stride);
      }
    }

    if (aligned != NULL)
    {
      rs2_release_frame(aligned);
    }
    rs2_release_frame(frames);

    if (published > 0 && published % STATS_EVERY_N == 0)
    {
      struct timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);
      double dt = (double)(now.tv_sec - win_start.tv_sec) +
                  (double)(now.tv_nsec - win_start.tv_nsec) / 1e9;
      if (dt > 0.0)
      {
        printf(LOG_PREFIX "已发布 %u 帧, %.1f fps\n", published, in_window / dt);
      }
      win_start = now;
      in_window = 0;
    }
  }

  printf(LOG_PREFIX "收到退出信号，共发布 %u 帧\n", published);
  exit_code = 0;

cleanup_rs:
  if (align_queue != NULL)
  {
    rs2_delete_frame_queue(align_queue);
  }
  if (align_block != NULL)
  {
    rs2_delete_processing_block(align_block);
  }
  if (profile != NULL)
  {
    rs2_delete_pipeline_profile(profile);
  }
  if (pipe != NULL)
  {
    rs2_error *se = NULL;
    rs2_pipeline_stop(pipe, &se);
    if (se != NULL)
    {
      rs2_free_error(se);
    }
    rs2_delete_pipeline(pipe);
  }
  if (cfg != NULL)
  {
    rs2_delete_config(cfg);
  }
  if (dev_list != NULL)
  {
    rs2_delete_device_list(dev_list);
  }
  if (rs_ctx != NULL)
  {
    rs2_delete_context(rs_ctx);
  }

cleanup_pubs:
  if (opt.enable_depth)
  {
    image_pub_fini(&depth_pub, &node);
  }
  image_pub_fini(&color_pub, &node);

cleanup_node:
  rcl_ignore(rcl_node_fini(&node));
  rcl_ignore(rcl_shutdown(&context));
  rcl_ignore(rcl_context_fini(&context));
  rcl_ignore(rcl_init_options_fini(&init_options));

  return exit_code;
}
