
---

## 🧠 项目目录结构说明（Markdown 格式优化版）

```
project/                          # 💡 仿真测试根目录
│
├─ core/                          # ✨ 领域核心（机型无关）
│   ├─ __init__.py                # 导出公共类，保持空亦可
│   ├─ controller.py              # 基本动作发送
│   ├─ state.py                   # 运行时状态
│   └─ scenario.py                # 生成/加载场景；调用 rosbag 播放或动态造障
│
├─ commands/                      # 🔌 业务插件层（常变）
│   ├─ __init__.py                # 注册中心：register()/get()/register_order()
│   ├─ base.py                    # Command 抽象 & 参数校验
│   ├─ order.py                   # order命令
│   └─ mapping/                   # 建图系列保留独立文件 —> 后续建议模块职责和命名优化
│       ├─ __init__.py            # 注册 mapping 命令
│       ├─ start_build_map.py     # 创建新地图
│       ├─ create_remote_region.py# 遥控建区域
│       ├─ create_app_region.py   # 下发建区域
│       └─ edit_map_element.py    # 编辑禁区/图案/通道/区域
│
├─ products/                      # 📦 机型 Profile & 能力声明
│   ├─ base.yaml                  # 通用默认配置
│   ├─ G2451.yaml                 # G2451 专属差异化
│   └─ G2422.yaml                 # G2422 专属差异化
│
├─ infrastructure/                # 🔧 技术细节封装（可替换扩展）
│   └─ ros/                       # ROS 适配层
│       ├─ __init__.py
│       ├─ pub.py                 # TopicPublisher
│       ├─ sub.py                 # TopicSubscriber
│       └─ bag_player.py          # Rosbag play 封装
│
├─ resources/                     # 📚资源包
│   └─ bags/                      # rosbag
│       ├─ reflectors/            # —— 反光贴数据（按产品）
│       │   ├─ G2422_reflector.bag# G2422 机型反光贴
│       │   └─ G2451_reflector.bag# G2451 机型反光贴
│       └─ obstacles/             # 🚧 障碍物场景数据
│           ├─ cone_corridor.bag        # 🚥 锥桶窄道
│           ├─ high_grass_patch.bag     # 🌱 高草区
│           ├─ charging_station.bag     # 🔋 回充基座 & 墙角
│           └─ dynamic_human.bag        # 🚶 移动人体干扰
│
├─ utils/                         # 🛠️ 通用工具
│   ├─ __init__.py
│   ├─ msg_codec.py               # 🔄 消息格式转换器
│   └─ logging.py                 # 📝 统一日志系统
│
└─ tests/                         # 内部测试 非测试单元
```

---

## 软件说明

1、主线程（Pytest 测试线程）

2、ROS线程

3、TimeManager 调度线程

---

---

## 🛠 环境设置与执行方式

### 设置 PYTHONPATH 示例

```bash
export PYTHONPATH="/home/zhumingfeng/Project/autoapicheck"
export PYTHONPATH="/data"

# 执行测试示例
python /home/zhumingfeng/Project/autoapicheck/project/tests/test_robot_motion.py 
python /data/project/tests/test_robot_motion.py 


export PYTHONPATH="/data/project:/data/python3/lib/python3.10:/data/python3/lib/python3.10/dist-packages:/application/lib/python3/dist-packages:${PYTHONPATH}"

export PYTHONPATH="/data/simulationpytest:/data/python3/lib/python3.10:/data/python3/lib/python3.10/dist-packages:/application/lib/python3/dist-packages:${PYTHONPATH}"
```

### 吴忌版本环境变量设置

```bash
export PYTHONPATH="/data/autoapicheck/project:/data/python3/lib/python3.10:/data/python3/lib/python3.10/dist-packages:/application/lib/python3/dist-packages:${PYTHONPATH}"
```

---

## 📦 Python 环境自带的包记录（简化展示）

```bash
zhumingfeng@movapc:/mnt/c/Users/zmaib/Download/python3.tar/python3$ tree -L 3
.
├── bin
│   ├── 2to3
│   ├── 2to3-3.10
│   ├── Activate.ps1
│   ├── activate
│   ├── activate.csh
│   ├── activate.fish
│   ├── idle3
│   ├── idle3.10
│   ├── pip
│   ├── pip3
│   ├── pip3.10
│   ├── pydoc3
│   ├── pydoc3.10
│   ├── python
│   ├── python3
│   ├── python3-config
│   ├── python3.10
│   └── python3.10-config
├── include
├── lib
│   ├── libpython3.10.so
│   ├── libpython3.10.so.1.0
│   ├── libpython3.so
│   ├── pkgconfig
│   │   ├── python-3.10-embed.pc
│   │   ├── python-3.10.pc
│   │   ├── python3-embed.pc
│   │   └── python3.pc
│   └── python3.10
│       ├── LICENSE.txt
│       ├── __future__.py
│       ├── __phello__.foo.py
│       ├── __pycache__
│       ├── _aix_support.py
│       ├── _bootsubprocess.py
│       ├── _collections_abc.py
│       ├── _compat_pickle.py
│       ├── _compression.py
│       ├── _markupbase.py
│       ├── _osx_support.py
│       ├── _py_abc.py
│       ├── _pydecimal.py
│       ├── _pyio.py
│       ├── _sitebuiltins.py
│       ├── _strptime.py
│       ├── _sysconfigdata.py
│       ├── _threading_local.py
│       ├── _weakrefset.py
│       ├── abc.py
│       ├── actionlib
│       ├── actionlib-1.12.1.egg-info
│       ├── actionlib_msgs
│       ├── aifc.py
│       ├── angles
│       ├── angles-1.9.12.egg-info
│       ├── antigravity.py
│       ├── argparse.py
│       ├── ast.py
│       ├── asynchat.py
│       ├── asyncio
│       ├── asyncore.py
│       ├── base64.py
│       ├── bdb.py
│       ├── binhex.py
│       ├── bisect.py
│       ├── bond
│       ├── bondpy
│       ├── bondpy-1.8.5.egg-info
│       ├── bz2.py
│       ├── cProfile.py
│       ├── calendar.py
│       ├── catkin
│       ├── catkin-0.7.29.egg-info
│       ├── catkin_pkg
│       ├── catkin_pkg-0.4.24.dist-info
│       ├── cgi.py
│       ├── cgitb.py
│       ├── chunk.py
│       ├── cmd.py
│       ├── code.py
│       ├── codecs.py
│       ├── codeop.py
│       ├── collections
│       ├── colorsys.py
│       ├── compileall.py
│       ├── concurrent
│       ├── configparser.py
│       ├── contextlib.py
│       ├── contextvars.py
│       ├── copy.py
│       ├── copyreg.py
│       ├── crypt.py
│       ├── csv.py
│       ├── ctypes
│       ├── curses
│       ├── dataclasses.py
│       ├── datetime.py
│       ├── dbm
│       ├── decimal.py
│       ├── defusedxml
│       ├── difflib.py
│       ├── dis.py
│       ├── dist-packages
│       ├── distutils
│       ├── doctest.py
│       ├── dynamic_reconfigure
│       ├── dynamic_reconfigure-1.6.5.egg-info
│       ├── email
│       ├── encodings
│       ├── ensurepip
│       ├── enum.py
│       ├── filecmp.py
│       ├── fileinput.py
│       ├── fnmatch.py
│       ├── fractions.py
│       ├── ftplib.py
│       ├── functools.py
│       ├── gencpp
│       ├── gencpp-0.6.5.egg-info
│       ├── genericpath.py
│       ├── genmsg
│       ├── genmsg-0.5.16.egg-info
│       ├── genpy
│       ├── genpy-0.6.16.egg-info
│       ├── geometry_msgs
│       ├── getopt.py
│       ├── getpass.py
│       ├── gettext.py
│       ├── glob.py
│       ├── graphlib.py
│       ├── gzip.py
│       ├── hashlib.py
│       ├── heapq.py
│       ├── hmac.py
│       ├── html
│       ├── http
│       ├── idlelib
│       ├── imaplib.py
│       ├── imghdr.py
│       ├── imp.py
│       ├── importlib
│       ├── inspect.py
│       ├── io.py
│       ├── ipaddress.py
│       ├── json
│       ├── keyword.py
│       ├── lib-dynload
│       ├── lib2to3
│       ├── linecache.py
│       ├── locale.py
│       ├── logging
│       ├── lzma.py
│       ├── mailbox.py
│       ├── mailcap.py
│       ├── message_filters
│       ├── message_filters-1.14.12.egg-info
│       ├── mimetypes.py
│       ├── modulefinder.py
│       ├── multiprocessing
│       ├── nav_msgs
│       ├── netrc.py
│       ├── nntplib.py
│       ├── nodelet
│       ├── nodelet_topic_tools
│       ├── ntpath.py
│       ├── nturl2path.py
│       ├── numbers.py
│       ├── opcode.py
│       ├── operator.py
│       ├── optparse.py
│       ├── os.py
│       ├── pathlib.py
│       ├── pcl_msgs
│       ├── pdb.py
│       ├── pickle.py
│       ├── pickletools.py
│       ├── pipes.py
│       ├── pkgutil.py
│       ├── platform.py
│       ├── plistlib.py
│       ├── poplib.py
│       ├── posixpath.py
│       ├── pprint.py
│       ├── profile.py
│       ├── pstats.py
│       ├── pty.py
│       ├── py_compile.py
│       ├── pyclbr.py
│       ├── pydoc.py
│       ├── pydoc_data
│       ├── pyparsing
│       ├── queue.py
│       ├── quopri.py
│       ├── random.py
│       ├── re.py
│       ├── reprlib.py
│       ├── rlcompleter.py
│       ├── ros
│       ├── rosbag
│       ├── rosbag-1.14.13.egg-info
│       ├── rosclean
│       ├── rosclean-1.14.9.egg-info
│       ├── roscpp
│       ├── rosgraph
│       ├── rosgraph-1.14.12.egg-info
│       ├── rosgraph_msgs
│       ├── roslaunch
│       ├── roslaunch-1.14.12.egg-info
│       ├── roslib
│       ├── roslib-1.14.9.egg-info
│       ├── roslz4
│       ├── roslz4-1.14.13.egg-info
│       ├── rosmaster
│       ├── rosmaster-1.14.12.egg-info
│       ├── rosmsg
│       ├── rosmsg-1.14.12.egg-info
│       ├── rosnode
│       ├── rosnode-1.14.12.egg-info
│       ├── rosparam
│       ├── rosparam-1.12.17.egg-info
│       ├── rospkg
│       ├── rospkg-1.3.0.dist-info
│       ├── rospy
│       ├── rospy-1.14.12.egg-info
│       ├── rosservice
│       ├── rosservice-1.14.12.egg-info
│       ├── rostopic
│       ├── rostopic-1.14.12.egg-info
│       ├── rosunit
│       ├── rosunit-1.14.9.egg-info
│       ├── runpy.py
│       ├── sched.py
│       ├── secrets.py
│       ├── selectors.py
│       ├── sensor_msgs
│       ├── sensor_msgs-1.12.8.egg-info
│       ├── shelve.py
│       ├── shlex.py
│       ├── shutil.py
│       ├── signal.py
│       ├── site-packages
│       ├── site.py
│       ├── smclib
│       ├── smclib-1.8.5.egg-info
│       ├── smtpd.py
│       ├── smtplib.py
│       ├── sndhdr.py
│       ├── socket.py
│       ├── socketserver.py
│       ├── sqlite3
│       ├── sre_compile.py
│       ├── sre_constants.py
│       ├── sre_parse.py
│       ├── ssl.py
│       ├── stat.py
│       ├── statistics.py
│       ├── std_msgs
│       ├── std_srvs
│       ├── string.py
│       ├── stringprep.py
│       ├── struct.py
│       ├── subprocess.py
│       ├── sunau.py
│       ├── symtable.py
│       ├── sysconfig.py
│       ├── tabnanny.py
│       ├── tarfile.py
│       ├── telnetlib.py
│       ├── tempfile.py
│       ├── textwrap.py
│       ├── tf
│       ├── tf-1.12.1.egg-info
│       ├── tf2_msgs
│       ├── tf2_py
│       ├── tf2_py-0.6.5.egg-info
│       ├── tf2_ros
│       ├── tf2_ros-0.6.5.egg-info
│       ├── this.py
│       ├── threading.py
│       ├── timeit.py
│       ├── tkinter
│       ├── token.py
│       ├── tokenize.py
│       ├── topic_tools
│       ├── topic_tools-1.14.13.egg-info
│       ├── trace.py
│       ├── traceback.py
│       ├── tracemalloc.py
│       ├── tty.py
│       ├── turtle.py
│       ├── turtledemo
│       ├── types.py
│       ├── typing.py
│       ├── unittest
│       ├── urllib
│       ├── uu.py
│       ├── uuid.py
│       ├── venv
│       ├── visualization_msgs
│       ├── warnings.py
│       ├── wave.py
│       ├── weakref.py
│       ├── webbrowser.py
│       ├── wsgiref
│       ├── xdrlib.py
│       ├── xml
│       ├── xmlrpc
│       ├── yaml
│       ├── zipapp.py
│       ├── zipfile.py
│       ├── zipimport.py
│       └── zoneinfo
├── lib64
└── pyvenv.cfg
```

---