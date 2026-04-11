# asr_sdm_zephyr

Zephyr 工程仓库，根目录提供环境安装脚本与项目目录组织。

## Install script
```sh
git clone https://github.com/AmpRobo/asr_sdm_zephyr.git
cd asr_sdm_zephyr
python3 install_zephyr.py
```

如果系统依赖已经准备完成，可以跳过 `apt` 安装：

```sh
python3 install_zephyr.py --skip-apt
```

安装完成后，加载环境：

```sh
source .venv/bin/activate
source zephyr_ws/env.sh
```

脚本默认会完成以下工作：

- 创建 Python 虚拟环境 `.venv`
- 拉取 `zephyr.repos` 中声明的 Zephyr 仓库
- 在 `zephyr_ws/` 下初始化 west 工作区
- 下载并安装 Zephyr SDK
- 生成环境脚本 `zephyr_ws/env.sh`

## Directory structure
```sh
asr_sdm_zephyr/
├── install_zephyr.py
├── zephyr.repos
├── modules/
│   └── drivers/
├── projects/
│   └── asr_sdm_screw_unit/
│       ├── boards/
│       │   └── xiao_rp2350_rp2350a_m33.overlay
│       ├── CMakeLists.txt
│       ├── Kconfig
│       ├── prj.conf
│       └── src/
│           └── main.c
├── .venv/
├── zephyr_ws/
└── build/
```

其中 `.venv/`、`zephyr_ws/` 和 `build/` 为本地安装或构建后生成的目录。

## Compilation
Build command example:
```sh
west build -p always -b xiao_rp2350/rp2350a/m33 -s projects/asr_sdm_screw_unit -d build/asr_sdm_screw_unit
```