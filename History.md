O
n.n.n / 2022-07-25
==================

  * rpmsg: checkout native module from msm-5.4
  * drivers: soc: qcom: Checkout to msm-5.4
  * kernel: power: Entirely checkout to msm-5.4
  * power_supply: don't allocate attrname
  * regulator: Retry read and write i2c operations on failure
  * version 12
  * arm64: dts: kona: Switch to step-wise thermal governor
  * BACKPORT: msm: kgsl: Add support for KGSL_PROP_VK_DEVICE_ID
  * BACKPORT: msm: kgsl: Add a property to query gpu model name
  * BACKPORT: msm: kgsl: Add support to get gpu model from device tree
  * version 11
  * rmnet_ipa: Fix netdev watchdog triggering on suspend
  * msm: ipa: Do not acquire wakelocks
  * defconfig: Enable ExFAT
  * exfat: initialize content to avoid warn
  * BACKPORT: fs: add sync_blockdev_range()
  * BACKPORT: exfat: reduce block requests when zeroing a cluster
  * BACKPORT: exfat: introduce mount option 'sys_tz'
  * BACKPORT: exfat: fix referencing wrong parent directory information after renaming
  * BACKPORT: exfat: do not clear VolumeDirty in writeback
  * BACKPORT: exfat: allow access to paths with trailing dots
  * BACKPORT: exfat: fix missing REQ_SYNC in exfat_update_bhs()
  * BACKPORT: exfat: remove argument 'sector' from exfat_get_dentry()
  * BACKPORT: exfat: move super block magic number to magic.h
  * BACKPORT: exfat: fix i_blocks for files truncated over 4 GiB
  * BACKPORT: exfat: reuse exfat_inode_info variable instead of calling EXFAT_I()
  * BACKPORT: exfat: make exfat_find_location() static
  * BACKPORT: exfat: fix typos in comments
  * BACKPORT: exfat: simplify is_valid_cluster()
  * BACKPORT: exfat: fix incorrect loading of i_blocks for large files
  * BACKPORT: exfat: handle wrong stream entry size in exfat_readdir()
  * BACKPORT: exfat: avoid incorrectly releasing for root inode
  * BACKPORT: exfat: speed up iterate/lookup by fixing start point of traversing cluster chain
  * BACKPORT: exfat: improve write performance when dirsync enabled
  * BACKPORT: exfat: add support ioctl and FITRIM function
  * BACKPORT: exfat: introduce bitmap_lock for cluster bitmap access
  * BACKPORT: exfat: fix erroneous discard when clear cluster bit
  * BACKPORT: exfat: improve performance of exfat_free_cluster when using dirsync mount option
  * BACKPORT: exfat: fix shift-out-of-bounds in exfat_fill_super()
  * BACKPORT: exfat: Avoid allocating upcase table using kcalloc()
  * exfat: remove useless check in exfat_move_file()
  * exfat: remove 'rwoffset' in exfat_inode_info
  * exfat: replace memcpy with structure assignment
  * exfat: remove useless directory scan in exfat_add_entry()
  * exfat: eliminate dead code in exfat_find()
  * exfat: use i_blocksize() to get blocksize
  * exfat: fix misspellings using codespell tool
  * exfat: fix use of uninitialized spinlock on error path
  * exfat: fix pointer error checking
  * exfat: retain 'VolumeFlags' properly
  * exfat: optimize exfat_zeroed_cluster()
  * exfat: add error check when updating dir-entries
  * exfat: write multiple sectors at once
  * exfat: remove EXFAT_SB_DIRTY flag
  * exfat: fix name_hash computation on big endian systems
  * exfat: fix wrong size update of stream entry by typo
  * exfat: fix wrong hint_stat initialization in exfat_find_dir_entry()
  * exfat: fix overflow issue in exfat_cluster_to_sector()
  * [PATCH] fs: Reduce boilerplate in fsid handling
  * fs: exfat: Import `5.8-2arter97` and include to build
  * drivers: android: Place a no-op PSI driver for SLMK
  * version 10
  * techpack: display:  Implement doze mode
  * techpack: display: remove stock doze implementation
  * arm64: dts: umi: optimise battery charging further
  * mbcache: Speed up cache entry creation
  * arm64: select HAVE_MOVE_PMD for faster mremap
  * mm: speed up mremap by 20x on large regions
  * ARM64: kona/lito: Optimize FOD HBM to fix fod flashing
  * techpack: display: temporary disable DC dimming when in fod_hbm mode
  * techpack: display: temporary disable DC dimming when in hbm mode
  * techpack: display: adapt exposure adjustment methods to sm8250
  * techpack: display: Let exposure adjustment switchable via sysfs
  * techpack: display: Introduce exposure adjustment driver
  * version 9
  * defconfig: regen
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
  * version 8
  * defconfigs: enable pelt halflife
  * ANDROID: sched/fair: correct pelt load information in sched-pelt.h
  * FROMLIST: sched/fair: add support to tune PELT ramp/decay timings
  * sched/fair: Fix kernel warning
  * UPSTREAM: sched: Fix out-of-bound access in uclamp
  * BACKPORT: sched/fair: Fix overutilized update in enqueue_task_fair()
  * kernel/sched: spread big prefer_idle tasks to little cores
  * UPSTREAM: sched/rt: Disable RT_RUNTIME_SHARE by default
  * sched: refine code for computing energy
  * sched/fair: refine can_migrate_boosted_task
  * sched/core: fix userspace affining threads incorrectly by task name.
  * kernel: sched: account for real time utilization
  * sched: Improve the scheduler
  * sched/core: Fix use after free issue in is_sched_lib_based_app()
  * sched/core: fix userspace affining threads incorrectly
  * sched: fine tune task placement for prioritized tasks
  * sched: fair: placement optimization for heavy load
  * Revert "sched: fine tune task placement for prioritized tasks"
  * sched: fine tune task placement for prioritized tasks
  * sched/fair: schedule lower priority tasks from little cores
  * sched/fair: do not use boosted margin for prefer_high_cap case
  * sched/fair: use actual cpu capacity to calculate boosted util
  * sched: separate capacity margin for boosted tasks
  * sched: separate boost signal from placement hint
  * kernel: sched: merge changes from LA.UM.9.12.R2.10.00.00.685.011
  * Use find_best_target to select cpu for a zero-util task
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * GKI: sched: Add back the root_domain.overutilized field
  * GKI: sched: Compile out push_task field in struct rq
  * sched: restrict iowait boost to tasks with prefer_idle
  * trace: sched: add capacity change tracing
  * sched: reduce softirq conflicts with RT
  * sched/fair: let scheduler skip util checking if cpu is idle
  * kernel: sched: Mitigate non-boosted tasks preempting boosted tasks
  * Revert "sched/core: fix userspace affining threads incorrectly"
  * Revert "sched/core: Fix use after free issue in is_sched_lib_based_app()"
  * Revert "sched: Improve the scheduler"
  * sched/fair: prefer exclusive mid cluster cpu for top-app task
  * sched: delete unused & buggy function definitions
  * sched/fair: fix implementation of is_min_capacity_cpu()
  * sched/fair: refine some scheduler changes from AU drop
  * BACKPORT: sched/fair: if sync flag ignored, try to place in mid cluster
  * Revert "sched/walt: Improve the scheduler"
  * sched: core: Disable double lock/unlock balance in move_queued_task()
  * sched: fair: Disable double lock/unlock balance in detach_task()
  * sched/fair: apply sync wake-up to pure CFS path
  * Revert "sched: fair: Always try to use energy efficient cpu for wakeups"
  * sched: fair: avoid little cpus due to sync, prev bias
  * sched: fix issue of cpu freq running at max always
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * kernel: sched: fix cpu cpu_capacity_orig being capped incorrectly
  * sched/walt: Fix negative count of sched_asym_cpucapacity static key
  * Revert "sched/fair: Add policy for restricting prefer_spread to newly idle balance"
  * ashmem: Adapt building on msm-4.19
  * ashmem: Rewrite to improve clarity and performance
  * version 7
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * cpufreqX: schedutil: Use kobject release() method to free sugov_tunables
  * arch: dts: Throttle charging speed and Optimise charging current
  * arm64: lib: Memory utilities optimization
  * version 6
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * setlocalversion: Never append a "+" to localversion
  * drivers:block:zram:default lz4
  * zram: Move default compression algorithm choice to Kconfig
  * defconfig: enable lz4 compression
  * version 5
  * version 4
  * version 3
  * build: version 2
  * build: its Snow time !!!
  * defconfig: Bump SLMK minfree & timeout
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * treewide: use mi drm notifier
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * techpack: camera: Import xiaomi changes from psyche-r-oss
  * msm: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * msm-camera: Unmap secure buffers in secure usecase
  * techpack: camera: cam_fd: fix double add into list
  * techpack: camera: ois: Add lc898128 firmware support
  * defconfig: regen
  * platform: msm: Fix dangerous relocation
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * defconfig: regen
  * add yarpiins build stuff
  * gitignore: add out directory
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * arch: arm64: configs: xiaomi: enable CONFIG_CPU_FREQ_STAT
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data
  * sysfs: Add sysfs_emit and sysfs_emit_at to format sysfs output
  * subsystem_restart: Always performs soft resets when subsystems crash
  * techpack: display: drm/msm: add idle state sysfs node
  * ARM64/dts: alioth: Re-order refresh rate timing
  * arm64/dts: alioth: Fix typo
  * ARM64/configs: alioth: Enable FPC fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/dts: alioth: Update display panel timing configuration
  * ARM64/dts: alioth: Fix rear camera eeprom entry
  * arch: arm64: configs: xiaomi: lmi: Enable back some drivers
  * arch: arm64: configs: xiaomi: lmi: Reenable CONFIG_DM_SNAPSHOT
  * arch: arm64: configs: xiaomi: Enable device-specific drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * techpack: display: Import Xiaomi changes
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: audio: Import Xiaomi changes
  * sound: soc: Import Xiaomi changes
  * kernel: Import Xiaomi changes
  * init: Import Xiaomi changes
  * fs: pstore: Import Xiaomi changes
  * drivers: video: backlight: Import Xiaomi changes
  * drivers: thermal: Import Xiaomi changes
  * drivers: staging: android: Import Xiaomi changes
  * drivers: spi: Import Xiaomi changes
  * drivers: soc: qcom: Import Xiaomi changes
  * Revert "regulator: qpnp-lcdb: Disable step voltage ramp for PM8150L V3"
  * Revert "regulator: qpnp-lcdb: Add n_voltages property for LCDB regulators"
  * power: qcom: Import xiaomi modifications from psyche-r-oss
  * smb5: Fix SDP current override for USB phy speed
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * drivers: net: Import Xiaomi changes
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * drivers: misc: Import DRV8846 motor driver
  * drivers: misc: Import AKM09970 driver
  * drivers: media: rc: Import Xiaomi SPI IR driver
  * drivers: leds: Import Xiaomi changes
  * drivers: leds: Import AW2015 driver
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch_mi: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * drivers: input: touchscreen: Prevent unnecessary input sync
  * drivers: input: touchscreen: Import Xiaomi drivers
  * drivers: input: misc: Import AW8697 haptic driver
  * drivers: input: fingerprint: Prevent unlock rejection timeout
  * drivers: input: Import Xiaomi fingerprint drivers
  * drivers: iio: proximity: Import Xiaomi Ultrasound driver
  * drivers: gpu: drm: Import Xiaomi changes
  * drivers: cpufreq: Import Xiaomi changes
  * firmware: Add FIRMWARE_IN_KERNEL support
  * arch: arm64: dts: panel: Disable mi smart fps
  * arch: arm64: dts: panel: Enable Qsync minimum refresh rate
  * arch: arm64: dts: panel: Cleanup supported refresh rates list
  * arch: arm64: dts: panel: Correct physical dimensions reporting
  * arch: arm64: dts: qcom: kona-camera: fix i2c and order clocks correctly
  * arch: arm64: dts: Update dts code to match stock dtbo
  * arch: arm64: dts: Import Xiaomi devicetree
  * arch: arm64: configs: xiaomi: Enable device-specific platform
  * arch: arm64: configs: xiaomi: Enable QCACLD
  * arch: arm64: configs: xiaomi: Enable CONFIG_BUILD_ARM64_DT_OVERLAY
  * arch: arm64: configs: xiaomi: Set localversion to lineageos
  * arch: arm64: configs: xiaomi: Initial import
  * platform: Add support for Xiaomi SM8250 platform
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * techpack: Integrate Qualcomm data-kernel package
  * techpack: audio: Make all konaauto drivers built in
  * techpack: audio: Correct symlinks
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * Merge tag 'LA.UM.9.12.r1-14400-SMxx50.0' of  https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0-rebase
  * Merge tag 'LA.UM.9.12.r1-14400-SMxx50.0' of  https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0-rebase
  * Merge tag 'LA.UM.9.12.r1-14400-SMxx50.0' of  https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0-rebase
  * Merge tag 'LA.UM.9.12.r1-14400-SMxx50.0' of  https://git.codelinaro.org/clo/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0-rebase
  * Merge tag 'LA.UM.9.12.r1-14400-SMxx50.0' of  https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0-rebase
  * Merge tag 'LA.UM.9.12.r1-14400-SMxx50.0' of  https://git.codelinaro.org/clo/la/platform/vendor/opensource/camera-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0-rebase
  * Merge tag 'LA.UM.9.12.r1-08000-SMxx50.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0-rebase
  * Merge tag 'LA.UM.9.12.r1-08000-SMxx50.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0-rebase
  * gitignore: Don't ignore techpack
  * techpack: Remove .gitignore
  * Merge tag 'LA.UM.9.12.r1-14400-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0-rebase
  * initial commit
  * Merge 34bb42eb71428965dc411f8bbc691e8280c84259 on remote branch
  * Merge "FROMGIT: cgroup: Use separate src/dst nodes when preloading css_sets for migration"
  * msm: ipahal: modify parameter from eq_bitfield[i] to i
  * Merge "soc: qcom: socinfo: Add support for Kona-7230 soc-id"
  * Merge "msm: ADSPRPC: Restrict untrusted applications from attaching to GuestOS"
  * soc: qcom: socinfo: Add support for Kona-7230 soc-id
  * Merge "qcom: genoa_extcon_notifier: Add genoa extcon notifier driver"
  * FROMGIT: cgroup: Use separate src/dst nodes when preloading css_sets for migration
  * Merge "msm: kgsl: fix division results for UBSAN warnings"
  * Merge "msm: camera: Add Page Read, DelayUSEC support in camera i2c"
  * msm: camera: Add Page Read, DelayUSEC support in camera i2c
  * msm: ADSPRPC: Restrict untrusted applications from attaching to GuestOS
  * Merge "HID: wacom: fix problems when device is not a valid USB device"
  * msm: kgsl: fix division results for UBSAN warnings
  * Merge "HID: add USB_HID dependancy on some USB HID drivers"
  * Merge "HID: add hid_is_usb() function to make it simpler for USB detection"
  * Merge "igmp: Add ip_mc_list lock in ip_check_mc_rcu"
  * HID: wacom: fix problems when device is not a valid USB device
  * Merge "ANDROID: selinux: modify RTM_GETNEIGH{TBL}"
  * HID: add USB_HID dependancy on some USB HID drivers
  * HID: add USB_HID dependancy to hid-chicony
  * HID: add USB_HID dependancy to hid-prodikeys
  * Merge "USB: gadget: validate interface OS descriptor requests"
  * Merge "UPSTREAM: usb: gadget: clear related members when goto fail"
  * ANDROID: selinux: modify RTM_GETNEIGH{TBL}
  * misc: updates to controller's driver
  * HID: add hid_is_usb() function to make it simpler for USB detection
  * Merge "lib/iov_iter: initialize "flags" in new pipe_buffer"
  * Merge "UPSTREAM: USB: gadget: validate interface OS descriptor requests"
  * Merge "UPSTREAM: net/packet: rx_owner_map depends on pg_vec"
  * Merge "UPSTREAM: mmc: block: fix read single on recovery logic"
  * Merge "defconfig: Enable KinecticsXR Nordic chip for SKU4"
  * Merge "smcinvoke : file private data validation which is sent by userspace"
  * USB: gadget: validate interface OS descriptor requests
  * UPSTREAM: usb: gadget: clear related members when goto fail
  * UPSTREAM: usb: gadget: don't release an existing dev->buf
  * igmp: Add ip_mc_list lock in ip_check_mc_rcu
  * UPSTREAM: net/packet: rx_owner_map depends on pg_vec
  * UPSTREAM: mmc: block: fix read single on recovery logic
  * UPSTREAM: USB: gadget: validate interface OS descriptor requests
  * UPSTREAM: usb: gadget: rndis: check size of RNDIS_MSG_SET command
  * lib/iov_iter: initialize "flags" in new pipe_buffer
  * Merge "mmc: sdhci-msm: Remove mmc->card check from MGPI design support"
  * Merge "msm: ipa: fix to NULL terminate the pointer"
  * Merge "coresight: Fix clang 14.x forbidden warnings/errors"
  * msm: ipa: fix to NULL terminate the pointer
  * mmc: sdhci-msm: Remove mmc->card check from MGPI design support
  * defconfig: Enable KinecticsXR Nordic chip for SKU4
  * misc: add makefile changes for Nordic chip
  * coresight: Fix clang 14.x forbidden warnings/errors
  * defconfig: msm: Enable DEBUG_INFO_DWARF4 for sdm660
  * smcinvoke : file private data validation which is sent by userspace
  * Merge 0bfc6c856bf196fe10ddb9b249543a9283054fce on remote branch
  * Merge 7d6b043520ec4d7d9574ce49de0357bb5713dd92 on remote branch
  * qcom: genoa_extcon_notifier: Add genoa extcon notifier driver
  * Merge 7e6242a76c9ada28629e35bc29bdc8c9950c1b59 on remote branch
  * msm: kgsl: Zap performance counters across context switches
  * msm: kgsl: Add a sysfs node to control performance counter reads
  * fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs
  * Merge 24aebadd512b27f8cb15a4b5ae634e104d74e183 on remote branch
  * Merge 1c29131bbaafca9e6ee6ed00bf330c9243c6d804 on remote branch
  * Merge fd4cdfacba48b59775c41ed00cbc28a63ead2026 on remote branch
  * msm: adsprpc: Handle UAF in process shell memory
  * Merge 82146398a44b8081ef002068af683b1f44f4b226 on remote branch
  * Merge 4bd53b8750ce1a2bb7e1e9567beba529e85503b8 on remote branch

n.n.n / 2022-07-12
==================

  * Merge tag 'LA.UM.9.12.1.r1-01900-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01900-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01900-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * defconfig: regen
  * Merge tag 'LA.UM.9.12.1.r1-01900-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * misc: updates to controller's driver
  * defconfig: Enable KinecticsXR Nordic chip for SKU4
  * misc: add makefile changes for Nordic chip
  * Merge 0aeda3450b36a69eee79b1321a4057982c7f9c33 on remote branch
  * BACKPORT: msm: kgsl: Add support for KGSL_PROP_VK_DEVICE_ID
  * BACKPORT: msm: kgsl: Add a property to query gpu model name
  * BACKPORT: msm: kgsl: Add support to get gpu model from device tree
  * arm64: dts: umi: remove Overclock overlay
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/camera-kernel
  * Revert "drivers: usb: pd: remove fix pd output for 5v"
  * alarmtimer: Minimize wakeup time
  * regulator: Retry read and write i2c operations on failure
  * Revert "regulator: qpnp-lcdb: Disable step voltage ramp for PM8150L V3"
  * Revert "regulator: qpnp-lcdb: Add n_voltages property for LCDB regulators"
  * power: qcom: Import xiaomi modifications from psyche-r-oss
  * power: qcom: Import xiaomi modifications from elish-r-oss
  * Merge "defconfig: msm: Add initial defconfig for Bengal-lite"
  * Merge "soc: qcom: Add retry mechanism to encryption related Secure Monitor calls"
  * version 11
  * cpuidle: lpm-levels: decrease reported deep idle latency
  * rmnet_ipa: Fix netdev watchdog triggering on suspend
  * msm: ipa: Do not acquire wakelocks
  * defconfig: Enable ExFAT
  * exfat: initialize content to avoid warn
  * BACKPORT: fs: add sync_blockdev_range()
  * BACKPORT: exfat: reduce block requests when zeroing a cluster
  * BACKPORT: exfat: introduce mount option 'sys_tz'
  * BACKPORT: exfat: fix referencing wrong parent directory information after renaming
  * BACKPORT: exfat: do not clear VolumeDirty in writeback
  * BACKPORT: exfat: allow access to paths with trailing dots
  * BACKPORT: exfat: fix missing REQ_SYNC in exfat_update_bhs()
  * BACKPORT: exfat: remove argument 'sector' from exfat_get_dentry()
  * BACKPORT: exfat: move super block magic number to magic.h
  * BACKPORT: exfat: fix i_blocks for files truncated over 4 GiB
  * BACKPORT: exfat: reuse exfat_inode_info variable instead of calling EXFAT_I()
  * BACKPORT: exfat: make exfat_find_location() static
  * BACKPORT: exfat: fix typos in comments
  * BACKPORT: exfat: simplify is_valid_cluster()
  * BACKPORT: exfat: fix incorrect loading of i_blocks for large files
  * BACKPORT: exfat: handle wrong stream entry size in exfat_readdir()
  * BACKPORT: exfat: avoid incorrectly releasing for root inode
  * BACKPORT: exfat: speed up iterate/lookup by fixing start point of traversing cluster chain
  * BACKPORT: exfat: improve write performance when dirsync enabled
  * BACKPORT: exfat: add support ioctl and FITRIM function
  * BACKPORT: exfat: introduce bitmap_lock for cluster bitmap access
  * BACKPORT: exfat: fix erroneous discard when clear cluster bit
  * BACKPORT: exfat: improve performance of exfat_free_cluster when using dirsync mount option
  * BACKPORT: exfat: fix shift-out-of-bounds in exfat_fill_super()
  * BACKPORT: exfat: Avoid allocating upcase table using kcalloc()
  * exfat: remove useless check in exfat_move_file()
  * exfat: remove 'rwoffset' in exfat_inode_info
  * exfat: replace memcpy with structure assignment
  * exfat: remove useless directory scan in exfat_add_entry()
  * exfat: eliminate dead code in exfat_find()
  * exfat: use i_blocksize() to get blocksize
  * exfat: fix misspellings using codespell tool
  * exfat: fix use of uninitialized spinlock on error path
  * exfat: fix pointer error checking
  * exfat: retain 'VolumeFlags' properly
  * exfat: optimize exfat_zeroed_cluster()
  * exfat: add error check when updating dir-entries
  * exfat: write multiple sectors at once
  * exfat: remove EXFAT_SB_DIRTY flag
  * exfat: fix name_hash computation on big endian systems
  * exfat: fix wrong size update of stream entry by typo
  * exfat: fix wrong hint_stat initialization in exfat_find_dir_entry()
  * exfat: fix overflow issue in exfat_cluster_to_sector()
  * [PATCH] fs: Reduce boilerplate in fsid handling
  * fs: exfat: Import `5.8-2arter97` and include to build
  * drivers: android: Place a no-op PSI driver for SLMK
  * drivers: thermal: cleanup thermal implementation
  * defconfigs: Enable garbage collector for userspace wakeup sources
  * BACKPORT: soc: qcom: smp2p_sleepstate: Add support for multiple clients
  * BACKPORT: soc: qcom: smp2p_sleepstate: Fix compilation warning
  * BACKPORT: soc: qcom: smp2p: Fix possible smp2p entry double free
  * BACKPORT: soc: qcom: smp2p: Add support for suspend to disk
  * BACKPORT: soc: qcom: smp2p: Log unchanged entries
  * Revert "qmi_rmnet: Make powersave workqueue unbound and freezable"
  * defconfig: msm: Add initial defconfig for Bengal-lite
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * mmc: sdhci-msm: Revert clear tuning done flag while hs400 tuning
  * version 10
  * techpack: display:  Implement doze mode
  * techpack: display: remove stock doze implementation
  * power: supply: revert caf upstream changes
  * kernel: power: Entirely checkout to msm-5.4
  * kernel: power: Checkout to msm-5.4
  * drivers: soc: qcom: Checkout to msm-5.4
  * arm64: dts: umi: optimise battery charging further
  * Revert "ARM64: dts: Disable Xiaomi FOD dimlayer impl"
  * msm: camera: Remove nonsensical register reads in cam_vfe_fe_reg_dump()
  * msm: camera: Fix memory leak in cam_res_mgr_probe()
  * msm-camera: Unmap secure buffers in secure usecase
  * mbcache: Speed up cache entry creation
  * arm64: select HAVE_MOVE_PMD for faster mremap
  * mm: speed up mremap by 20x on large regions
  * ARM64: kona/lito: Optimize FOD HBM to fix fod flashing
  * smb5: report fast charging when a proprietary charger is attached
  * Merge "diag: Update log and event mask code ranges"
  * Merge "add controller's drivers"
  * Merge "platform: msm: usb_bam: Fix potential use-after-free in connect_pipe"
  * Merge "defconfig: msm: Enable LEDS for QCS2290"
  * Merge "tzlog : Add check to avoid null pointer dereferencing"
  * defconfig: msm: Enable LEDS for QCS2290
  * platform: msm: usb_bam: Fix potential use-after-free in connect_pipe
  * tzlog : Add check to avoid null pointer dereferencing
  * soc: qcom: Add retry mechanism to encryption related Secure Monitor calls
  * Merge "msm: adsprpc: Fix compat ioctl invoke failure"
  * Merge "msm: ipa3: Avoid device crash while SSR"
  * Merge "msm: kgsl: Remove 'fd' dependency to get dma_buf handle"
  * Merge 5f40eefaf7c85daf81f7e228152226fe4601b1fe on remote branch
  * add controller's drivers
  * msm: kgsl: Remove 'fd' dependency to get dma_buf handle
  * msm: adsprpc: Fix compat ioctl invoke failure
  * diag: Update log and event mask code ranges
  * smp: Fix smp_call_function_single_async prototype
  * Makefile: Move -Wno-unused-but-set-variable out of GCC only block
  * msm: ipa3: Avoid device crash while SSR
  * techpack: display: temporary disable DC dimming when in fod_hbm mode
  * techpack: display: temporary disable DC dimming when in hbm mode
  * techpack: display: adapt exposure adjustment methods to sm8250
  * techpack: display: Let exposure adjustment switchable via sysfs
  * techpack: display: Introduce exposure adjustment driver
  * msm: kgsl: Use copy_struct_from_user() helper
  * version 9
  * defconfig: regen
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * techpack: Checkout camera kernel to MIcode psyche-r-oss
  * revert"ARM64: dts: Remove address from node-name for reserved-mem regions"
  * revert"ARM64: dts: Correct SLPI/ADSP/SPSS/CDSP memory regions on LMI/CMI/UMI"
  * Merge "msm: ADSPRPC: Update unsigned pd support on cDSP from kernel"
  * Merge "BACKPORT: dmabuf: fix use-after-free of dmabuf's file->f_inode"
  * Merge "lib: introduce copy_struct_from_user() helper"
  * Merge "msm: synx: release reference holding the lock"
  * msm: ADSPRPC: Update unsigned pd support on cDSP from kernel
  * version 8
  * ARM64: kona-v2.1: fix max frequency on thermals
  * ARM64: dts: xiaomi-sm8250-common: configure msm-thermal-simple
  * Revert "ARM64: dts: umi-sm8250: configure msm-thermal-simple"
  * arm64: dts: xiaomi: Allow big cluster to idle in USB perf mode
  * defconfigs: enable pelt halflife
  * ANDROID: sched/fair: correct pelt load information in sched-pelt.h
  * FROMLIST: sched/fair: add support to tune PELT ramp/decay timings
  * sched/fair: Fix kernel warning
  * UPSTREAM: sched: Fix out-of-bound access in uclamp
  * BACKPORT: sched/fair: Fix overutilized update in enqueue_task_fair()
  * kernel/sched: spread big prefer_idle tasks to little cores
  * UPSTREAM: sched/rt: Disable RT_RUNTIME_SHARE by default
  * sched: refine code for computing energy
  * sched/fair: refine can_migrate_boosted_task
  * sched/core: fix userspace affining threads incorrectly by task name.
  * kernel: sched: account for real time utilization
  * sched: Improve the scheduler
  * sched/core: Fix use after free issue in is_sched_lib_based_app()
  * sched/core: fix userspace affining threads incorrectly
  * sched: fine tune task placement for prioritized tasks
  * sched: fair: placement optimization for heavy load
  * Revert "sched: fine tune task placement for prioritized tasks"
  * sched: fine tune task placement for prioritized tasks
  * sched/fair: schedule lower priority tasks from little cores
  * sched/fair: do not use boosted margin for prefer_high_cap case
  * sched/fair: use actual cpu capacity to calculate boosted util
  * sched: separate capacity margin for boosted tasks
  * sched: separate boost signal from placement hint
  * kernel: sched: merge changes from LA.UM.9.12.R2.10.00.00.685.011
  * Use find_best_target to select cpu for a zero-util task
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * GKI: sched: Add back the root_domain.overutilized field
  * GKI: sched: Compile out push_task field in struct rq
  * sched: restrict iowait boost to tasks with prefer_idle
  * trace: sched: add capacity change tracing
  * sched: reduce softirq conflicts with RT
  * sched/fair: let scheduler skip util checking if cpu is idle
  * kernel: sched: Mitigate non-boosted tasks preempting boosted tasks
  * Revert "sched/core: fix userspace affining threads incorrectly"
  * Revert "sched/core: Fix use after free issue in is_sched_lib_based_app()"
  * Revert "sched: Improve the scheduler"
  * sched/fair: prefer exclusive mid cluster cpu for top-app task
  * sched: delete unused & buggy function definitions
  * sched/fair: fix implementation of is_min_capacity_cpu()
  * sched/fair: refine some scheduler changes from AU drop
  * BACKPORT: sched/fair: if sync flag ignored, try to place in mid cluster
  * Revert "sched/walt: Improve the scheduler"
  * sched: core: Disable double lock/unlock balance in move_queued_task()
  * sched: fair: Disable double lock/unlock balance in detach_task()
  * sched/fair: apply sync wake-up to pure CFS path
  * Revert "sched: fair: Always try to use energy efficient cpu for wakeups"
  * sched: fair: avoid little cpus due to sync, prev bias
  * sched: fix issue of cpu freq running at max always
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * kernel: sched: fix cpu cpu_capacity_orig being capped incorrectly
  * sched/walt: Fix negative count of sched_asym_cpucapacity static key
  * Revert "sched/fair: Add policy for restricting prefer_spread to newly idle balance"
  * fs: ext4: fsync: optimize double-fsync() a bunch
  * block : makefile : disable align mismatch
  * blk-mq: fix corruption with direct issue
  * ashmem: Adapt building on msm-4.19
  * ashmem: Rewrite to improve clarity and performance
  * block: cfq-iosched: Port samsung optimizations from SM-N986B
  * cpufreq: schedutil: Smoothen WALT predicted load boosting
  * cpufreq: schedutil: fix check for stale utilization values
  * sched: Do not reduce perceived CPU capacity while idle
  * sched/features: Fix hrtick reprogramming
  * sched: features: Disable EAS_PREFER_IDLE
  * arch: arm64: dts: qcom: kona-camera: fix i2c and order clocks correctly
  * techpack: display: Store dc status when fod hbm enabled
  * techpack: display: dsi_phy: Force allow panel phy power off
  * techpack: display: dsi_panel: Force enable Ultra-Low Power State feature for all panel
  * version 7
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * sysfs: Add sysfs_emit and sysfs_emit_at to format sysfs output
  * Revert "Add toggle for disabling newly added USB devices"
  * cpufreqX: schedutil: Use kobject release() method to free sugov_tunables
  * Merge "msm: kgsl: Remove unnecessary NULL checks"
  * Merge "msm: kgsl: Assign a default value to the variable ret"
  * dfc: bearer based QMAP powersave
  * Merge "fbdev: msm: check for valid fence before using objects"
  * Merge "sched: Improve the scheduler"
  * BACKPORT: dmabuf: fix use-after-free of dmabuf's file->f_inode
  * Merge "sched: walt: Improve the scheduler"
  * Merge 7cc7d569297d73062f58d860957f7a4edfc91033 on remote branch
  * sched: Improve the scheduler
  * lib: introduce copy_struct_from_user() helper
  * fbdev: msm: check for valid fence before using objects
  * f2fs: avoid infinite loop to flush node pages
  * f2fs: fix wrong condition check when failing metapage read
  * f2fs: remove unnecessary f2fs_lock_op in f2fs_new_inode
  * f2fs: fix to do sanity check on .cp_pack_total_block_count
  * f2fs: compress: fix to print raw data size in error path of lz4 decompression
  * f2fs: remove redundant parameter judgment
  * drivers: input: aw8697: Fix missmerge
  * ARM64: dts: Use new awinic haptics configuration
  * input: misc: aw8697_haptic: Upgrade from zeus-s-oss
  * arch: dts: Throttle charging speed and Optimise charging current
  * msm: thermal: simple: account for battery temperature in calculations
  * arm64: lib: Memory utilities optimization
  * proc: do not let google camera run in the background
  * Merge "diag: Use correct size while reallocating for hdlc encoding"
  * Merge "qseecom : qseecom_scale_bus_bandwidth doesn't check the negative mode"
  * Merge "soc: qcom: mdt_loader: Replacing sprintf with snprintf"
  * Merge "defconfig: arm: msm: Added spmi_sdam support for SDM429"
  * msm:ipa3: Fixed pointer dereference issue without checking for null
  * msm: kgsl: Assign a default value to the variable ret
  * msm: kgsl: Remove unnecessary NULL checks
  * diag: Use correct size while reallocating for hdlc encoding
  * Merge "tasks, sched/core: Ensure tasks are available for a grace period after leaving the runqueue"
  * msm: synx: release reference holding the lock
  * sched: walt: Improve the scheduler
  * Merge "defconfig: Enable UAC1 config for kona"
  * tasks, sched/core: Ensure tasks are available for a grace period after leaving the runqueue
  * tasks: Add a count of task RCU users
  * Merge "usb: misc: Add snapshot of diag_ipc_bridge driver"
  * Merge "usb: misc: ks_bridge: Add snapshot of ks_bridge driver"
  * usb: misc: Add snapshot of diag_ipc_bridge driver
  * Merge "defconfig: msm: Enable LED for QCS2290"
  * usb: misc: ks_bridge: Add snapshot of ks_bridge driver
  * Merge "msm: adsprpc: Wait for actual shutdown to complete"
  * Merge "coresight: Replacing sprintf with scnprintf"
  * msm: adsprpc: Wait for actual shutdown to complete
  * defconfig: msm: Enable LED for QCS2290
  * qseecom : qseecom_scale_bus_bandwidth doesn't check the negative mode
  * Merge "defconfig: kona: Enable mcp25xxfd driver for perf_defconfig"
  * coresight: Replacing sprintf with scnprintf
  * Merge "drivers: can: Enclose mcp25xxfd_dump_regs into CONFIG_DEBUG_FS"
  * defconfig: kona: Enable mcp25xxfd driver for perf_defconfig
  * drivers: can: Enclose mcp25xxfd_dump_regs into CONFIG_DEBUG_FS
  * msm: adsprpc:  Fix double fetch from fastrpc HLOS driver
  * soc: qcom: mdt_loader: Replacing sprintf with snprintf
  * Merge 7e6242a76c9ada28629e35bc29bdc8c9950c1b59 on remote branch
  * Merge "msm: adsprpc: Do length check to avoid arbitrary memory access"
  * msm: adsprpc: Do length check to avoid arbitrary memory access
  * Merge "pci_iomap: fix page fault issue on vmalloc with section mapping"
  * Merge "aio: fix use-after-free due to missing POLLFREE handling"
  * Merge "FROMGIT: USB: gadget: bRequestType is a bitfield, not a enum"
  * Merge "aio: keep poll requests on waitqueue until completed"
  * Merge "sctp: add param size validation for SCTP_PARAM_SET_PRIMARY"
  * Merge "binder: use wake_up_pollfree()"
  * Merge "fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs"
  * thermal: Increase thermal trip points to 16
  * f2fs: use spin_lock to avoid hang
  * f2fs: remove unnecessary read for F2FS_FITS_IN_INODE
  * f2fs: avoid an infinite loop in f2fs_sync_dirty_inodes
  * f2fs: fix to do sanity check on curseg->alloc_type
  * f2fs: fix to avoid potential deadlock
  * f2fs: fix missing free nid in f2fs_handle_failed_inode
  * f2fs: add a way to limit roll forward recovery time
  * f2fs: adjust readahead block number during recovery
  * f2fs: expose discard related parameters in sysfs
  * f2fs: move discard parameters into discard_cmd_control
  * f2fs: do not allow partial truncation on pinned file
  * icnss: Add code to pass device configs to wlan driver
  * fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs
  * msm: kgsl: Zap performance counters across context switches
  * msm: kgsl: Add a sysfs node to control performance counter reads
  * disp: uapi: drm: Add new blend type for CAC
  * Merge "msm: npu: Avoid buffer overflow when handling get_property packet"
  * Merge "USB: uac1: Fix audio quality issues for UAC1"
  * Merge "msm_bus: Check if cldata->pdata is null before access it"
  * version 6
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * USB: uac1: Fix audio quality issues for UAC1
  * defconfig: Enable UAC1 config for kona
  * UAC1: Add configfs attribute for changing ep max packet size
  * USB: f_uac1: Add check before accessing members of uac1
  * USB: f_uac1: add iad descriptor for UAC1 driver
  * f_uac1: Remove unwanted descriptors for volume/mute support
  * f_uac1: Add support for volume/mute control settings
  * msm: npu: Avoid buffer overflow when handling get_property packet
  * sctp: add param size validation for SCTP_PARAM_SET_PRIMARY
  * sctp: validate chunk size in __rcv_asconf_lookup
  * sctp: add size validation when walking chunks
  * sctp: validate from_addr_param return
  * aio: fix use-after-free due to missing POLLFREE handling
  * aio: keep poll requests on waitqueue until completed
  * signalfd: use wake_up_pollfree()
  * binder: use wake_up_pollfree()
  * wait: add wake_up_pollfree()
  * FROMGIT: USB: gadget: bRequestType is a bitfield, not a enum
  * UPSTREAM: USB: gadget: zero allocate endpoint 0 buffers
  * UPSTREAM: USB: gadget: detect too-big endpoint 0 requests
  * Merge "cnss2: Handle event processing after shutdown"
  * Merge "soc: qcom: socinfo: Add soc information for KhajeP and KhajeQ"
  * Merge "i2c: i2c-msm-v2: Set frequency parameters to INT and validate"
  * Merge d09499e9c8a99763bf9afa92c06601070523dc3e on remote branch
  * Merge "drivers: thermal: validate cdev sysfs state request before using it"
  * soc: qcom: socinfo: Add soc information for KhajeP and KhajeQ
  * msm_bus: Check if cldata->pdata is null before access it
  * spi: spi-msm-geni: Put device to suspend if PM status is active
  * Merge "cfg80211: Add support to advertize OCV support"
  * Merge "spi: spi-msm-geni: Protect from register access in suspend state"
  * Merge "spi: spi-msm-geni: Keep device to suspend if PM call fails"
  * cnss2: Handle event processing after shutdown
  * cfg80211: Add support to advertize OCV support
  * i2c: i2c-msm-v2: Set frequency parameters to INT and validate
  * msm: kgsl: Reduce latency while processing ioctls
  * setlocalversion: Never append a "+" to localversion
  * drivers:block:zram:default lz4
  * zram: Move default compression algorithm choice to Kconfig
  * defconfig: enable lz4 compression
  * drivers: thermal: validate cdev sysfs state request before using it
  * defconfigs: disable msm performance
  * USB: f_fs: Fix disconnect check during ongoing IO
  * Merge "msm: diag: fix copyright"
  * Merge "drivers: soc: qcom: Initialize blocking notifier as per lockdep"
  * msm: diag: fix copyright
  * Merge "diag: Validate the dci client before sending dci packet"
  * diag: Validate the dci client before sending dci packet
  * drivers: soc: qcom: Initialize blocking notifier as per lockdep
  * version 5
  * defconfig: regen
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * usb: gadget: u_audio: fix race condition on endpoint stop
  * usb: gadget: u_audio: Free requests only after callback
  * defconfig: Enable usb peripheral audio on SXR2130
  * Merge "diag: Ensure dci entry is valid before sending the packet"
  * Merge "msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()"
  * msm: ADSPRPC: Fix to avoid Use after free in fastrpc_init_process
  * Merge "msm: synx: fix copyright"
  * Merge "msm: ipa3: Fix to validate the NAT table entries during NAT table init"
  * msm: synx: fix copyright
  * clk: qcom: vdd-level: Update the vdd level for CX on Khaje
  * msm: ipa3: Fix to validate the NAT table entries during NAT table init
  * Merge 533e678218e41df69501158ff6eb84deafb6771c on remote branch
  * msm: synx: fix copyright
  * clk: qcom: gpucc: Update the vdd level for CX on Khaje
  * diag: Ensure dci entry is valid before sending the packet
  * Merge "usb: pd: Process request message as soon as it is received"
  * usb: pd: Process request message as soon as it is received
  * Merge "dfc: reset tx queue"
  * mmc: sdhci-msm: Update the MGPI SW check to avoid irq timeout
  * msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()
  * Merge "msm: ipa3: fix to cleanup the dma allocation"
  * Merge "msm: synx: remove synx handle details from logging"
  * dfc: reset tx queue
  * clk: qcom: gpucc: Add support for higher frequency for Khaje
  * Merge "clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje"
  * msm: ipa3: fix to cleanup the dma allocation
  * Merge "include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner"
  * clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje
  * Merge "msm: adsprpc: Handle UAF in fastrpc debugfs read"
  * include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner
  * Merge "usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind"
  * usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind
  * version 4
  * firmware: focaltech: Upgrade FW from haydin
  * touchscreen: focaltech_spi: Upgrade regardless of FW version check
  * techpack: audio: Mark msm-dai-q6 drivers as sync probe
  * techpack/audio: kona: Report correct key code of headset buttons
  * thermal: qcom: bcl: Return before thermal trip evaluates
  * defconfigs: enable Simple MSM Thermal
  * ARM64: dts: umi-sm8250: configure msm-thermal-simple
  * msm: thermal: simple: Introduce simple MSM thermal solution [msm-4.14]
  * version 3
  * drivers: thermal: limits-dcvs: Always build driver
  * drivers: power: supply: Disable battery capacity learning
  * defconfig: revert back to 100Hz tick
  * build: version 2
  * build: its Snow time !!!
  * drivers: usb: dwc3: Specify sync probe for dwc3-of-simple.
  * drivers: usb: dwc3: Specify sync probe for msm-dwc3 driver
  * drivers: usb: dwc3: Specify sync probe for dwc3 driver
  * drivers: usb: pd: remove fix pd output for 5v
  * Revert "usb: pd: Register typec partner in case AAA is connected"
  * PM / sleep: Skip OOM killer toggles when kernel is compiled for Android
  * drm: msm: optimize interpolation
  * techpack: drm: sde: fix a race condition
  * platform: msm: Fix dangerous relocation
  * defconfig: Bump SLMK minfree & timeout
  * power: process: Use lesser time to enter sleep
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * qos: Don't allow userspace to impose restrictions on CPU idle levels
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * dts: kona-gpu: Remove qos active latency node
  * treewide: use mi drm notifier
  * drivers: power: add timeouts to wakelocks
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * ARM64: umi: overlay gpu data from kona-v2-gpu.dtsi
  * arm64: dts: kona-v2-gpu: Overclock to 670mhz
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * gitignore: add out directory
  * add yarpiins build stuff
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/defconfig: apollo: Bringup custom defconfig
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * Revert "scsi: ufs: increase query timeout"
  * Revert "drivers: Add support for memory fragmentation test simulator driver"
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * ANDROID: sched: Exempt paused CPU from nohz idle balance
  * ARM64: configs: Make configs required to pass CTS tests mandatory
  * ARM64/defconfig: umi: Disable LSE atomics
  * ARM64/defconfig: umi: Use a timer frequency of 100 Hz
  * arm64: Inline the spin lock function family
  * ARM64/defconfig: umi: Disable unused errata
  * ARM64/defconfig: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * UPSTREAM: arm64: link with -z norelro for LLD or aarch64-elf
  * techpack: display: msm: dsi: set doze brightness for aod correctly
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64/defconfig: umi: Disable some unuse drivers
  * ARM64/defconfig: umi: Disable stability debug configs Disable the following stability debug configs for shipping: - CONFIG_EDAC_KRYO_ARM64_PANIC_ON_UE
  * ARM64/defconfig: umi: Enable LLD, RELR, Clang ThinLTO optimizations
  * ANDROID: sched: EAS: take cstate into account when selecting idle core
  * ARM64/defconfig: umi: Enable jump label
  * arm64/kernel: jump_label: Switch to relative references
  * locking/static_key: Add support for deferred static branches
  * jump_label: Fix NULL dereference bug in __jump_label_mod_update()
  * jump_label: Annotate entries that operate on __init code earlier
  * jump_label: Implement generic support for relative references
  * jump_label: Abstract jump_entry member accessors
  * jump_label: Use static_key_linked() accessor
  * mm/vmalloc.c: switch to WARN_ON() and move it under unlink_va()
  * mm/vmalloc.c: get rid of one single unlink_va() when merge
  * mm/vmalloc.c: preload a CPU with one object for split purpose
  * mm/vmalloc.c: remove "node" argument
  * mm/vmalloc.c: fix typo in comment
  * vmalloc: export __vmalloc_node_range for CONFIG_TEST_VMALLOC_MODULE
  * mm/vmalloc: pass VM_USERMAP flags directly to __vmalloc_node_range()
  * mm/vmalloc: do not call kmemleak_free() on not yet accounted memory
  * mm/vmalloc.c: make vmalloc_32_user() align base kernel virtual address to SHMLBA
  * mm: convert totalram_pages and totalhigh_pages variables to atomic
  * vfree: add debug might_sleep()
  * mm/vmalloc.c: improve vfree() kerneldoc
  * drivers: scsi: ufs: Remove unused d_lu_wb_buf_alloc
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64/defconfig: umi: Disable QHEE kernel memory protection
  * qcacld-3.0: Free a bunch of pkts at once
  * Makefile: Use O3 optimization level for Clang LTO
  * Makefile: Use -O3 optimization level
  * drivers: thermal: Don't qualify thermal polling as high priority
  * ARM64/defconfig: umi: Enable power efficient workqueues
  * ARM64/defconfig: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64/defconfig: Disable serial console support
  * arm64: Don't build legacy QCOM DTS.
  * rcu: fix a performance regression
  * qcacmn: Fix build error with !IPA_OFFLOAD
  * ARM64/defconfig: umi: Disable qti core control and sched autogroup
  * ARM: dts: kona: Disable IRQ debugging
  * ARM64/defconfig: umi: Disable some debug drivers
  * f2fs: Demote GC thread to idle scheduler class
  * f2fs: Set ioprio of GC kthread to idle
  * f2fs: Enlarge min_fsync_blocks to 20
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * UPSTREAM: zram: move backing_dev under macro CONFIG_ZRAM_WRITEBACK
  * UPSTREAM: zram: fix broken page writeback
  * UPSTREAM: zram: fix return value on writeback_store
  * UPSTREAM: zram: support page writeback
  * BACKPORT: zcomp: Use ARRAY_SIZE() for backends list
  * BACKPORT: zram: Allocate struct zcomp_strm as per-CPU memory
  * Revert "zram: introduce zram_entry to prepare dedup functionality"
  * Revert "zram: implement deduplication in zram"
  * Revert "zram: make deduplication feature optional"
  * Revert "zram: compare all the entries with same checksum for deduplication"
  * Revert "zram: fix race condition while returning zram_entry refcount"
  * ASoC: pcm: Add 24bit playback audio support
  * ARM64: configs: enable CONFIG_WIREGUARD
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * arm64/defconfig: umi: Enable crypto LZ4
  * arm64/defconfig: umi: Enable zram-writeback support
  * ARM64: umi/defconfigs: Bringup custom config
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
  * block: zram: Fix idle/writeback string compare
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * drivers: gpu: msm: Only build adreno 6xx part
  * video: backlight: disable modules enabled by default
  * media: gspca: disable gspca module
  * net: disable bridge netfilter module
  * net: ipv4: disable modules which are enabled by default
  * qcacmn: Fix suspicious string concatenation warning in fwlog
  * rmnet_shs: Fix CFI violation in packet assignment
  * rmnet_perf: Fix CFI violation in packet deaggregation
  * ntfs: Fix ntfs_test_inode and ntfs_init_locked_inode function type If the kernel is built with CFI we hit a __cfi_check_fail while mounting a partition
  * power: supply: Classify Battery Monitor Systems as batteries
  * cpufreq: stats: Replace the global lock with atomic.
  * techpack: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * techpack: camera: Unmap secure buffers in secure usecase
  * subsystem_restart: Always performs soft resets when subsystems crash
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * platform: msm: gsi: Export symbols only if compiled as module
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * scsi: ufs: Force enable write booster feature on UFS 3.1
  * scsi: ufs: Disable write booster feature support on UFS 2.2
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * treewide: Remove all Android.mk files
  * techpack: audio: Silence some logspam
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: Remove xiaomi early fingerprint wakeup optimization
  * audio: swr-mstr-ctrl: Fix unbalanced IRQ condition in swrm_runtime_suspend()
  * ARM64: configs: Enable XFRM_MIGRATE
  * ARM64: configs: Remove some debugging related features
  * ARM64: configs: Enable ARM64 Crypto Extensions SM3/4 and CRC32
  * ARM64: configs: Enable NTFS Filesystem
  * ARM64: configs: Enable ExFAT Filesystem
  * msm-poweroff: Store restart reason in panic
  * printk: Increase kernel log buffer size
  * smb5: Fix SDP current override for USB phy speed
  * mm: Disable watermark boost feature for K4.19
  * ARM64: configs: Skip file system sync in suspend
  * ARM64: configs: Disable PASR on kona devices
  * ARM64: configs: Reduce ION pool fill mark to 48MB
  * ARM64: configs: Build seperate DTB/DTBOs
  * ARM64: configs: Concatenate dtbs inside kernel image
  * ARM64: configs: Build Compressed Kernel Image
  * ARM64: configs: Build Qualcomm Atheros CLD WLAN module
  * ARM64: configs: Enable CPU frequency transition statistics
  * ARM64: configs: thyme: Enable support for block device writeback throttling
  * ARM64: configs: lmi: Enable AW2015 RGB LED support
  * ARM64: configs: lmi: Enable camera motor drivers
  * ARM64: configs: cas: Enable thermal dimming
  * ARM64: configs: apollo,cas: Enable panic on oops
  * ARM64: configs: apollo: Enable GPIO testing mode driver
  * ARM64: configs: Increase FRAME_WARN size to 4096
  * ARM64: configs: Enable Dynamic printk support
  * ARM64: configs: Disable MSM 11AD chip support
  * ARM64: configs: Enable xiaomi touchfeature driver
  * ARM64: configs: Disable USB CDC ADM support
  * ARM64: configs: Disable APP Armor security
  * ARM64: configs: Enable Ultrasound proximity driver
  * ARM64: configs: Enable USB Remote NDIS support
  * ARM64: configs: Disable USB Serial converter support
  * ARM64: configs: Disable V4L2 Videobuf2 Core
  * ARM64: configs: Enable SPI connected IR LED support
  * ARM64: configs: Enable PM8008 regulator driver
  * ARM64: configs: Enable device specific power supply drivers
  * ARM64: configs: Enable high performance logchar driver
  * ARM64: configs: Enable fastboot oem uart-enable command support
  * ARM64: configs: Disable support for Virtual terminal
  * ARM64: configs: Enable device specific fingerprint drivers
  * ARM64: configs: Enable AWINIC AW8697 haptics driver
  * ARM64: configs: Disable QTI Haptics support
  * ARM64: configs: Enable device specific touchscreen drivers
  * ARM64: configs: Disable Open-channel SSD target support
  * ARM64: configs: Enable CNSS QCA6390 chipset support
  * ARM64: configs: Enable QTI Smart Link Aggregation driver
  * ARM64: configs: Enable NAT, PEDIT and CSUM classifier Actions
  * ARM64: configs: Disable DEBUG_FS
  * ARM64: configs: Build Wilocity wil6210 driver
  * ARM64: configs: Enable Memory Technology Device (MTD) support
  * ARM64: configs: Disable Modules signature
  * ARM64: configs: Enable pstore support
  * ARM64: configs: Increase Kernel log buffer size
  * ARM64: configs: Enable Audit support
  * ARM64: configs: Enable device specific board platform
  * ARM64: configs: Generate xiaomi sm8250 devices configs from kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * bpf: Get the comm hash of the socket process stored inside sk buffer
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
  * fs: fuse: Implement FUSE passthrough
  * disp: msm: sde: Force SDE fd to start from 1
  * firmware: Import xiaomi touchscreens firmware
  * video: backlight: qcom-spmi-wled: Disable CABC on low brightness
  * video: backlight: Add support for thermal backlight dimming on J1S
  * video: backlight: Create brightness clone sysfs node
  * video: backlight: Notify on brightness node change
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * staging: android: ion: adjust system heap pool orders
  * staging: android: ion: Add xiaomi ion camera heap modifications
  * spi: gen-qcom: increase transfer timeout
  * soc: qcom: Add socinfo support for xiaomi sm8250 and sm7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
  * scsi: ufs: increase query timeout
  * mtd: devices: block2mtd: Sync scsi I/O on panic
  * debug: Add support to dump kmsg logs in kernel panic/oops using mtdoops
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * pinctrl: msm: Disable access to specified reserved gpios
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * cnss2: Add support for loading different bdwlan firmwares
  * net: cnss2: Add paramter to disable NV MAC support
  * net: cnss2: Increase cnss-daemon qmi timeout
  * media: rc: Add xiaomi modifications to IR driver
  * media: msm: cvp: Replace fence request handler kthread with workqueue
  * drivers: leds: Allow switching between dual flashlight leds
  * drivers: leds: Add support for AWINIC AW2015 3 Channel LED driver
  * drivers: cpuidle: Add xiaomi parameter to disable cpuidle and lpm
  * drivers: iio: Add support for Xiaomi ultrasound proximity sensor
  * drm: msm: Import xiaomi drm modifications
  * thermal: bcl_pmic5: Apply xiaomi thermal modifications
  * cpufreq: qcom: Always report maximum value of freq in limits_mitigation_notify()
  * drivers: thermal: Switch to CPUFREQ_ADJUST
  * mhi: mhi_qcom: Apply xiaomi modifications to sdx55 modem
  * drivers: misc: Import xiaomi GPIO testing mode driver
  * drivers: misc: Import Texas Instruments H-Bridge Stepper Motor Driver
  * drivers: misc: Import AKM Sensortecs AK09970 HALL sensor driver
  * block: Disable preemption before request_fn calls
  * ARM64: dts: qcom: Import kona mtp devicetree
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * input: touchscreen: Add support for xiaomi touchfeature dirver
  * input: touchscreen: Add support for Novatek NT36672c SPI touchscreen driver
  * input: touchscreen: Add support for GOODIX GT9886 touchscreen driver
  * input: touchscreen: Add support for STM FTS521 touchscreen driver
  * input: touchscreen: Add support for FocalTech ft8719/ft5452 touchscreen drivers
  * input: touchscreen: focaltech_spi: Don't ignore firmware files
  * input: touchscreen: Add support for FocalTech ft3658 touchscreen driver
  * input: touchscreen: Disable caf default touchscreen drivers
  * input: misc: aw8697_haptic: Add support to reload firmware for umi
  * input: misc: Add support for AWINIC AW8696 Haptic driver
  * input: fingerprint: Add support for GOODIX FOD fingerprint driver
  * input: fingerprint: Add support for GOODIX fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * drivers: Add support for memory fragmentation test simulator driver
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * techpack: audio: nuke xlogchar
  * Merge commit '99646a7c5d9584934073bd5c49438993f44c2f31' into android11-base
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * Add 'arch/arm64/boot/dts/vendor/' from commit 'bd2d6b0afa1f8aba19e41ce3bc29c16745595efa'
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * techpack: data: Build high performance ipa/rmnet drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * Revert "selinux: Relocate ss_initialized and selinux_enforcing to separate 4k"
  * ipa3: fix improper size checks
  * msm: ipa: Fix Makefile
  * kbuild: Remove gcc-wrapper
  * add toggle for disabling newly added USB devices
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * techpack/pinctrl-lpi: initialise at late_initcall
  * techpack: audio: makefile: do not export all the variables
  * techpack: audio: Correct symlinks
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into lineage-19.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/camera-kernel into 'LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-00300-XXXX.QSSI12.0
  * msm: adsprpc: Handle UAF in fastrpc debugfs read
  * usb: gadget: f_hid: Fix unbind issues
  * usb: gadget: f_uvc: Fix unbind issues
  * Merge "clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI"
  * msm: synx: remove synx handle details from logging
  * clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI
  * Merge "input: misc: qcom-power-on: Add support to log KPDPWR status"
  * Merge "msm: kgsl: Fix gpuaddr_in_range() to check upper bound"
  * mmc: sdhci-msm: Ensure SD card power isn't ON when card removed
  * Merge "clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL"
  * msm: kgsl: Fix gpuaddr_in_range() to check upper bound
  * Merge "usb: max-3421: Prevent corruption of freed memory"
  * clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL
  * spi: spi-msm-geni: Protect from register access in suspend state
  * spi: spi-msm-geni: Keep device to suspend if PM call fails
  * Merge "regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling"
  * Merge "seq_file: disallow extremely large seq buffer allocations"
  * regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling
  * Merge a425c3837db7a914ad21e6f5634b4c6dc7083b43 on remote branch
  * usb: gadget: f_uac1: Add support for UAC1 function
  * Merge "tzlog: update struct to get normal and fatal diag logs"
  * Merge "fs: crypto: Maintain reference count for class keys"
  * input: misc: qcom-power-on: Add support to log KPDPWR status
  * tzlog: update struct to get normal and fatal diag logs
  * firmware: qcom: Remove garbage characters from qsee log
  * firmware: qcom: add enlarged qsee log support
  * firmware: qcom: encrypted tz and qsee log support
  * fs: crypto: Maintain reference count for class keys
  * defconfig: arm: msm: Added spmi_sdam support for SDM429
  * usb: max-3421: Prevent corruption of freed memory
  * seq_file: disallow extremely large seq buffer allocations
  * pci_iomap: fix page fault issue on vmalloc with section mapping
  * Merge commit '82146398a44b8081ef002068af683b1f44f4b226' into kernel.lnx.4.19.r23-rel
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * defconfig: Enable Novatek NT36xxx Touch for tron target
  * cnss2: Update bound checks for sbl reg dumps to SRAM mem range

n.n.n / 2022-06-06
==================

  * version 11
  * cpuidle: lpm-levels: decrease reported deep idle latency
  * rmnet_ipa: Fix netdev watchdog triggering on suspend
  * msm: ipa: Do not acquire wakelocks
  * defconfig: Enable ExFAT
  * exfat: initialize content to avoid warn
  * BACKPORT: fs: add sync_blockdev_range()
  * BACKPORT: exfat: reduce block requests when zeroing a cluster
  * BACKPORT: exfat: introduce mount option 'sys_tz'
  * BACKPORT: exfat: fix referencing wrong parent directory information after renaming
  * BACKPORT: exfat: do not clear VolumeDirty in writeback
  * BACKPORT: exfat: allow access to paths with trailing dots
  * BACKPORT: exfat: fix missing REQ_SYNC in exfat_update_bhs()
  * BACKPORT: exfat: remove argument 'sector' from exfat_get_dentry()
  * BACKPORT: exfat: move super block magic number to magic.h
  * BACKPORT: exfat: fix i_blocks for files truncated over 4 GiB
  * BACKPORT: exfat: reuse exfat_inode_info variable instead of calling EXFAT_I()
  * BACKPORT: exfat: make exfat_find_location() static
  * BACKPORT: exfat: fix typos in comments
  * BACKPORT: exfat: simplify is_valid_cluster()
  * BACKPORT: exfat: fix incorrect loading of i_blocks for large files
  * BACKPORT: exfat: handle wrong stream entry size in exfat_readdir()
  * BACKPORT: exfat: avoid incorrectly releasing for root inode
  * BACKPORT: exfat: speed up iterate/lookup by fixing start point of traversing cluster chain
  * BACKPORT: exfat: improve write performance when dirsync enabled
  * BACKPORT: exfat: add support ioctl and FITRIM function
  * BACKPORT: exfat: introduce bitmap_lock for cluster bitmap access
  * BACKPORT: exfat: fix erroneous discard when clear cluster bit
  * BACKPORT: exfat: improve performance of exfat_free_cluster when using dirsync mount option
  * BACKPORT: exfat: fix shift-out-of-bounds in exfat_fill_super()
  * BACKPORT: exfat: Avoid allocating upcase table using kcalloc()
  * exfat: remove useless check in exfat_move_file()
  * exfat: remove 'rwoffset' in exfat_inode_info
  * exfat: replace memcpy with structure assignment
  * exfat: remove useless directory scan in exfat_add_entry()
  * exfat: eliminate dead code in exfat_find()
  * exfat: use i_blocksize() to get blocksize
  * exfat: fix misspellings using codespell tool
  * exfat: fix use of uninitialized spinlock on error path
  * exfat: fix pointer error checking
  * exfat: retain 'VolumeFlags' properly
  * exfat: optimize exfat_zeroed_cluster()
  * exfat: add error check when updating dir-entries
  * exfat: write multiple sectors at once
  * exfat: remove EXFAT_SB_DIRTY flag
  * exfat: fix name_hash computation on big endian systems
  * exfat: fix wrong size update of stream entry by typo
  * exfat: fix wrong hint_stat initialization in exfat_find_dir_entry()
  * exfat: fix overflow issue in exfat_cluster_to_sector()
  * [PATCH] fs: Reduce boilerplate in fsid handling
  * fs: exfat: Import `5.8-2arter97` and include to build
  * drivers: android: Place a no-op PSI driver for SLMK
  * drivers: thermal: cleanup thermal implementation
  * defconfigs: Enable garbage collector for userspace wakeup sources
  * BACKPORT: soc: qcom: smp2p_sleepstate: Add support for multiple clients
  * BACKPORT: soc: qcom: smp2p_sleepstate: Fix compilation warning
  * BACKPORT: soc: qcom: smp2p: Fix possible smp2p entry double free
  * BACKPORT: soc: qcom: smp2p: Add support for suspend to disk
  * BACKPORT: soc: qcom: smp2p: Log unchanged entries
  * Revert "qmi_rmnet: Make powersave workqueue unbound and freezable"
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * version 10
  * techpack: display:  Implement doze mode
  * techpack: display: remove stock doze implementation
  * power: supply: revert caf upstream changes
  * kernel: power: Entirely checkout to msm-5.4
  * kernel: power: Checkout to msm-5.4
  * drivers: soc: qcom: Checkout to msm-5.4
  * arm64: dts: umi: optimise battery charging further
  * Revert "ARM64: dts: Disable Xiaomi FOD dimlayer impl"
  * msm: camera: Remove nonsensical register reads in cam_vfe_fe_reg_dump()
  * msm: camera: Fix memory leak in cam_res_mgr_probe()
  * msm-camera: Unmap secure buffers in secure usecase
  * mbcache: Speed up cache entry creation
  * arm64: select HAVE_MOVE_PMD for faster mremap
  * mm: speed up mremap by 20x on large regions
  * ARM64: kona/lito: Optimize FOD HBM to fix fod flashing
  * smb5: report fast charging when a proprietary charger is attached
  * Merge 5f40eefaf7c85daf81f7e228152226fe4601b1fe on remote branch
  * smp: Fix smp_call_function_single_async prototype
  * Makefile: Move -Wno-unused-but-set-variable out of GCC only block
  * techpack: display: temporary disable DC dimming when in fod_hbm mode
  * techpack: display: temporary disable DC dimming when in hbm mode
  * techpack: display: adapt exposure adjustment methods to sm8250
  * techpack: display: Let exposure adjustment switchable via sysfs
  * techpack: display: Introduce exposure adjustment driver
  * msm: kgsl: Use copy_struct_from_user() helper
  * version 9
  * defconfig: regen
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * techpack: Checkout camera kernel to MIcode psyche-r-oss
  * revert"ARM64: dts: Remove address from node-name for reserved-mem regions"
  * revert"ARM64: dts: Correct SLPI/ADSP/SPSS/CDSP memory regions on LMI/CMI/UMI"
  * Merge "msm: ADSPRPC: Update unsigned pd support on cDSP from kernel"
  * Merge "BACKPORT: dmabuf: fix use-after-free of dmabuf's file->f_inode"
  * Merge "lib: introduce copy_struct_from_user() helper"
  * Merge "msm: synx: release reference holding the lock"
  * msm: ADSPRPC: Update unsigned pd support on cDSP from kernel
  * version 8
  * ARM64: kona-v2.1: fix max frequency on thermals
  * ARM64: dts: xiaomi-sm8250-common: configure msm-thermal-simple
  * Revert "ARM64: dts: umi-sm8250: configure msm-thermal-simple"
  * arm64: dts: xiaomi: Allow big cluster to idle in USB perf mode
  * defconfigs: enable pelt halflife
  * ANDROID: sched/fair: correct pelt load information in sched-pelt.h
  * FROMLIST: sched/fair: add support to tune PELT ramp/decay timings
  * sched/fair: Fix kernel warning
  * UPSTREAM: sched: Fix out-of-bound access in uclamp
  * BACKPORT: sched/fair: Fix overutilized update in enqueue_task_fair()
  * kernel/sched: spread big prefer_idle tasks to little cores
  * UPSTREAM: sched/rt: Disable RT_RUNTIME_SHARE by default
  * sched: refine code for computing energy
  * sched/fair: refine can_migrate_boosted_task
  * sched/core: fix userspace affining threads incorrectly by task name.
  * kernel: sched: account for real time utilization
  * sched: Improve the scheduler
  * sched/core: Fix use after free issue in is_sched_lib_based_app()
  * sched/core: fix userspace affining threads incorrectly
  * sched: fine tune task placement for prioritized tasks
  * sched: fair: placement optimization for heavy load
  * Revert "sched: fine tune task placement for prioritized tasks"
  * sched: fine tune task placement for prioritized tasks
  * sched/fair: schedule lower priority tasks from little cores
  * sched/fair: do not use boosted margin for prefer_high_cap case
  * sched/fair: use actual cpu capacity to calculate boosted util
  * sched: separate capacity margin for boosted tasks
  * sched: separate boost signal from placement hint
  * kernel: sched: merge changes from LA.UM.9.12.R2.10.00.00.685.011
  * Use find_best_target to select cpu for a zero-util task
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * GKI: sched: Add back the root_domain.overutilized field
  * GKI: sched: Compile out push_task field in struct rq
  * sched: restrict iowait boost to tasks with prefer_idle
  * trace: sched: add capacity change tracing
  * sched: reduce softirq conflicts with RT
  * sched/fair: let scheduler skip util checking if cpu is idle
  * kernel: sched: Mitigate non-boosted tasks preempting boosted tasks
  * Revert "sched/core: fix userspace affining threads incorrectly"
  * Revert "sched/core: Fix use after free issue in is_sched_lib_based_app()"
  * Revert "sched: Improve the scheduler"
  * sched/fair: prefer exclusive mid cluster cpu for top-app task
  * sched: delete unused & buggy function definitions
  * sched/fair: fix implementation of is_min_capacity_cpu()
  * sched/fair: refine some scheduler changes from AU drop
  * BACKPORT: sched/fair: if sync flag ignored, try to place in mid cluster
  * Revert "sched/walt: Improve the scheduler"
  * sched: core: Disable double lock/unlock balance in move_queued_task()
  * sched: fair: Disable double lock/unlock balance in detach_task()
  * sched/fair: apply sync wake-up to pure CFS path
  * Revert "sched: fair: Always try to use energy efficient cpu for wakeups"
  * sched: fair: avoid little cpus due to sync, prev bias
  * sched: fix issue of cpu freq running at max always
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * kernel: sched: fix cpu cpu_capacity_orig being capped incorrectly
  * sched/walt: Fix negative count of sched_asym_cpucapacity static key
  * Revert "sched/fair: Add policy for restricting prefer_spread to newly idle balance"
  * fs: ext4: fsync: optimize double-fsync() a bunch
  * block : makefile : disable align mismatch
  * blk-mq: fix corruption with direct issue
  * ashmem: Adapt building on msm-4.19
  * ashmem: Rewrite to improve clarity and performance
  * block: cfq-iosched: Port samsung optimizations from SM-N986B
  * cpufreq: schedutil: Smoothen WALT predicted load boosting
  * cpufreq: schedutil: fix check for stale utilization values
  * sched: Do not reduce perceived CPU capacity while idle
  * sched/features: Fix hrtick reprogramming
  * sched: features: Disable EAS_PREFER_IDLE
  * arch: arm64: dts: qcom: kona-camera: fix i2c and order clocks correctly
  * techpack: display: Store dc status when fod hbm enabled
  * techpack: display: dsi_phy: Force allow panel phy power off
  * techpack: display: dsi_panel: Force enable Ultra-Low Power State feature for all panel
  * version 7
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * sysfs: Add sysfs_emit and sysfs_emit_at to format sysfs output
  * Revert "Add toggle for disabling newly added USB devices"
  * cpufreqX: schedutil: Use kobject release() method to free sugov_tunables
  * Merge "msm: kgsl: Remove unnecessary NULL checks"
  * Merge "msm: kgsl: Assign a default value to the variable ret"
  * dfc: bearer based QMAP powersave
  * Merge "fbdev: msm: check for valid fence before using objects"
  * Merge "sched: Improve the scheduler"
  * BACKPORT: dmabuf: fix use-after-free of dmabuf's file->f_inode
  * Merge "sched: walt: Improve the scheduler"
  * Merge 7cc7d569297d73062f58d860957f7a4edfc91033 on remote branch
  * sched: Improve the scheduler
  * lib: introduce copy_struct_from_user() helper
  * fbdev: msm: check for valid fence before using objects
  * f2fs: avoid infinite loop to flush node pages
  * f2fs: fix wrong condition check when failing metapage read
  * f2fs: remove unnecessary f2fs_lock_op in f2fs_new_inode
  * f2fs: fix to do sanity check on .cp_pack_total_block_count
  * f2fs: compress: fix to print raw data size in error path of lz4 decompression
  * f2fs: remove redundant parameter judgment
  * drivers: input: aw8697: Fix missmerge
  * ARM64: dts: Use new awinic haptics configuration
  * input: misc: aw8697_haptic: Upgrade from zeus-s-oss
  * arch: dts: Throttle charging speed and Optimise charging current
  * msm: thermal: simple: account for battery temperature in calculations
  * arm64: lib: Memory utilities optimization
  * proc: do not let google camera run in the background
  * Merge "diag: Use correct size while reallocating for hdlc encoding"
  * Merge "qseecom : qseecom_scale_bus_bandwidth doesn't check the negative mode"
  * Merge "soc: qcom: mdt_loader: Replacing sprintf with snprintf"
  * Merge "defconfig: arm: msm: Added spmi_sdam support for SDM429"
  * msm:ipa3: Fixed pointer dereference issue without checking for null
  * msm: kgsl: Assign a default value to the variable ret
  * msm: kgsl: Remove unnecessary NULL checks
  * diag: Use correct size while reallocating for hdlc encoding
  * Merge "tasks, sched/core: Ensure tasks are available for a grace period after leaving the runqueue"
  * msm: synx: release reference holding the lock
  * sched: walt: Improve the scheduler
  * Merge "defconfig: Enable UAC1 config for kona"
  * tasks, sched/core: Ensure tasks are available for a grace period after leaving the runqueue
  * tasks: Add a count of task RCU users
  * Merge "usb: misc: Add snapshot of diag_ipc_bridge driver"
  * Merge "usb: misc: ks_bridge: Add snapshot of ks_bridge driver"
  * usb: misc: Add snapshot of diag_ipc_bridge driver
  * Merge "defconfig: msm: Enable LED for QCS2290"
  * usb: misc: ks_bridge: Add snapshot of ks_bridge driver
  * Merge "msm: adsprpc: Wait for actual shutdown to complete"
  * Merge "coresight: Replacing sprintf with scnprintf"
  * msm: adsprpc: Wait for actual shutdown to complete
  * defconfig: msm: Enable LED for QCS2290
  * qseecom : qseecom_scale_bus_bandwidth doesn't check the negative mode
  * Merge "defconfig: kona: Enable mcp25xxfd driver for perf_defconfig"
  * coresight: Replacing sprintf with scnprintf
  * Merge "drivers: can: Enclose mcp25xxfd_dump_regs into CONFIG_DEBUG_FS"
  * defconfig: kona: Enable mcp25xxfd driver for perf_defconfig
  * drivers: can: Enclose mcp25xxfd_dump_regs into CONFIG_DEBUG_FS
  * msm: adsprpc:  Fix double fetch from fastrpc HLOS driver
  * soc: qcom: mdt_loader: Replacing sprintf with snprintf
  * Merge 7e6242a76c9ada28629e35bc29bdc8c9950c1b59 on remote branch
  * Merge "msm: adsprpc: Do length check to avoid arbitrary memory access"
  * msm: adsprpc: Do length check to avoid arbitrary memory access
  * Merge "pci_iomap: fix page fault issue on vmalloc with section mapping"
  * Merge "aio: fix use-after-free due to missing POLLFREE handling"
  * Merge "FROMGIT: USB: gadget: bRequestType is a bitfield, not a enum"
  * Merge "aio: keep poll requests on waitqueue until completed"
  * Merge "sctp: add param size validation for SCTP_PARAM_SET_PRIMARY"
  * Merge "binder: use wake_up_pollfree()"
  * Merge "fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs"
  * thermal: Increase thermal trip points to 16
  * f2fs: use spin_lock to avoid hang
  * f2fs: remove unnecessary read for F2FS_FITS_IN_INODE
  * f2fs: avoid an infinite loop in f2fs_sync_dirty_inodes
  * f2fs: fix to do sanity check on curseg->alloc_type
  * f2fs: fix to avoid potential deadlock
  * f2fs: fix missing free nid in f2fs_handle_failed_inode
  * f2fs: add a way to limit roll forward recovery time
  * f2fs: adjust readahead block number during recovery
  * f2fs: expose discard related parameters in sysfs
  * f2fs: move discard parameters into discard_cmd_control
  * f2fs: do not allow partial truncation on pinned file
  * icnss: Add code to pass device configs to wlan driver
  * fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs
  * msm: kgsl: Zap performance counters across context switches
  * msm: kgsl: Add a sysfs node to control performance counter reads
  * disp: uapi: drm: Add new blend type for CAC
  * Merge "msm: npu: Avoid buffer overflow when handling get_property packet"
  * Merge "USB: uac1: Fix audio quality issues for UAC1"
  * Merge "msm_bus: Check if cldata->pdata is null before access it"
  * version 6
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * USB: uac1: Fix audio quality issues for UAC1
  * defconfig: Enable UAC1 config for kona
  * UAC1: Add configfs attribute for changing ep max packet size
  * USB: f_uac1: Add check before accessing members of uac1
  * USB: f_uac1: add iad descriptor for UAC1 driver
  * f_uac1: Remove unwanted descriptors for volume/mute support
  * f_uac1: Add support for volume/mute control settings
  * msm: npu: Avoid buffer overflow when handling get_property packet
  * sctp: add param size validation for SCTP_PARAM_SET_PRIMARY
  * sctp: validate chunk size in __rcv_asconf_lookup
  * sctp: add size validation when walking chunks
  * sctp: validate from_addr_param return
  * aio: fix use-after-free due to missing POLLFREE handling
  * aio: keep poll requests on waitqueue until completed
  * signalfd: use wake_up_pollfree()
  * binder: use wake_up_pollfree()
  * wait: add wake_up_pollfree()
  * FROMGIT: USB: gadget: bRequestType is a bitfield, not a enum
  * UPSTREAM: USB: gadget: zero allocate endpoint 0 buffers
  * UPSTREAM: USB: gadget: detect too-big endpoint 0 requests
  * Merge "cnss2: Handle event processing after shutdown"
  * Merge "soc: qcom: socinfo: Add soc information for KhajeP and KhajeQ"
  * Merge "i2c: i2c-msm-v2: Set frequency parameters to INT and validate"
  * Merge d09499e9c8a99763bf9afa92c06601070523dc3e on remote branch
  * Merge "drivers: thermal: validate cdev sysfs state request before using it"
  * soc: qcom: socinfo: Add soc information for KhajeP and KhajeQ
  * msm_bus: Check if cldata->pdata is null before access it
  * spi: spi-msm-geni: Put device to suspend if PM status is active
  * Merge "cfg80211: Add support to advertize OCV support"
  * Merge "spi: spi-msm-geni: Protect from register access in suspend state"
  * Merge "spi: spi-msm-geni: Keep device to suspend if PM call fails"
  * cnss2: Handle event processing after shutdown
  * cfg80211: Add support to advertize OCV support
  * i2c: i2c-msm-v2: Set frequency parameters to INT and validate
  * msm: kgsl: Reduce latency while processing ioctls
  * setlocalversion: Never append a "+" to localversion
  * drivers:block:zram:default lz4
  * zram: Move default compression algorithm choice to Kconfig
  * defconfig: enable lz4 compression
  * drivers: thermal: validate cdev sysfs state request before using it
  * defconfigs: disable msm performance
  * USB: f_fs: Fix disconnect check during ongoing IO
  * Merge "msm: diag: fix copyright"
  * Merge "drivers: soc: qcom: Initialize blocking notifier as per lockdep"
  * msm: diag: fix copyright
  * Merge "diag: Validate the dci client before sending dci packet"
  * diag: Validate the dci client before sending dci packet
  * drivers: soc: qcom: Initialize blocking notifier as per lockdep
  * version 5
  * defconfig: regen
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * usb: gadget: u_audio: fix race condition on endpoint stop
  * usb: gadget: u_audio: Free requests only after callback
  * defconfig: Enable usb peripheral audio on SXR2130
  * Merge "diag: Ensure dci entry is valid before sending the packet"
  * Merge "msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()"
  * msm: ADSPRPC: Fix to avoid Use after free in fastrpc_init_process
  * Merge "msm: synx: fix copyright"
  * Merge "msm: ipa3: Fix to validate the NAT table entries during NAT table init"
  * msm: synx: fix copyright
  * clk: qcom: vdd-level: Update the vdd level for CX on Khaje
  * msm: ipa3: Fix to validate the NAT table entries during NAT table init
  * Merge 533e678218e41df69501158ff6eb84deafb6771c on remote branch
  * msm: synx: fix copyright
  * clk: qcom: gpucc: Update the vdd level for CX on Khaje
  * diag: Ensure dci entry is valid before sending the packet
  * Merge "usb: pd: Process request message as soon as it is received"
  * usb: pd: Process request message as soon as it is received
  * Merge "dfc: reset tx queue"
  * mmc: sdhci-msm: Update the MGPI SW check to avoid irq timeout
  * msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()
  * Merge "msm: ipa3: fix to cleanup the dma allocation"
  * Merge "msm: synx: remove synx handle details from logging"
  * dfc: reset tx queue
  * clk: qcom: gpucc: Add support for higher frequency for Khaje
  * Merge "clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje"
  * msm: ipa3: fix to cleanup the dma allocation
  * Merge "include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner"
  * clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje
  * Merge "msm: adsprpc: Handle UAF in fastrpc debugfs read"
  * include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner
  * Merge "usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind"
  * usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind
  * version 4
  * firmware: focaltech: Upgrade FW from haydin
  * touchscreen: focaltech_spi: Upgrade regardless of FW version check
  * techpack: audio: Mark msm-dai-q6 drivers as sync probe
  * techpack/audio: kona: Report correct key code of headset buttons
  * thermal: qcom: bcl: Return before thermal trip evaluates
  * defconfigs: enable Simple MSM Thermal
  * ARM64: dts: umi-sm8250: configure msm-thermal-simple
  * msm: thermal: simple: Introduce simple MSM thermal solution [msm-4.14]
  * version 3
  * drivers: thermal: limits-dcvs: Always build driver
  * drivers: power: supply: Disable battery capacity learning
  * defconfig: revert back to 100Hz tick
  * build: version 2
  * build: its Snow time !!!
  * drivers: usb: dwc3: Specify sync probe for dwc3-of-simple.
  * drivers: usb: dwc3: Specify sync probe for msm-dwc3 driver
  * drivers: usb: dwc3: Specify sync probe for dwc3 driver
  * drivers: usb: pd: remove fix pd output for 5v
  * Revert "usb: pd: Register typec partner in case AAA is connected"
  * PM / sleep: Skip OOM killer toggles when kernel is compiled for Android
  * drm: msm: optimize interpolation
  * techpack: drm: sde: fix a race condition
  * platform: msm: Fix dangerous relocation
  * defconfig: Bump SLMK minfree & timeout
  * power: process: Use lesser time to enter sleep
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * qos: Don't allow userspace to impose restrictions on CPU idle levels
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * dts: kona-gpu: Remove qos active latency node
  * treewide: use mi drm notifier
  * drivers: power: add timeouts to wakelocks
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * ARM64: umi: overlay gpu data from kona-v2-gpu.dtsi
  * arm64: dts: kona-v2-gpu: Overclock to 670mhz
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * gitignore: add out directory
  * add yarpiins build stuff
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/defconfig: apollo: Bringup custom defconfig
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * Revert "scsi: ufs: increase query timeout"
  * Revert "drivers: Add support for memory fragmentation test simulator driver"
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * ANDROID: sched: Exempt paused CPU from nohz idle balance
  * ARM64: configs: Make configs required to pass CTS tests mandatory
  * ARM64/defconfig: umi: Disable LSE atomics
  * ARM64/defconfig: umi: Use a timer frequency of 100 Hz
  * arm64: Inline the spin lock function family
  * ARM64/defconfig: umi: Disable unused errata
  * ARM64/defconfig: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * UPSTREAM: arm64: link with -z norelro for LLD or aarch64-elf
  * techpack: display: msm: dsi: set doze brightness for aod correctly
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64/defconfig: umi: Disable some unuse drivers
  * ARM64/defconfig: umi: Disable stability debug configs Disable the following stability debug configs for shipping: - CONFIG_EDAC_KRYO_ARM64_PANIC_ON_UE
  * ARM64/defconfig: umi: Enable LLD, RELR, Clang ThinLTO optimizations
  * ANDROID: sched: EAS: take cstate into account when selecting idle core
  * ARM64/defconfig: umi: Enable jump label
  * arm64/kernel: jump_label: Switch to relative references
  * locking/static_key: Add support for deferred static branches
  * jump_label: Fix NULL dereference bug in __jump_label_mod_update()
  * jump_label: Annotate entries that operate on __init code earlier
  * jump_label: Implement generic support for relative references
  * jump_label: Abstract jump_entry member accessors
  * jump_label: Use static_key_linked() accessor
  * mm/vmalloc.c: switch to WARN_ON() and move it under unlink_va()
  * mm/vmalloc.c: get rid of one single unlink_va() when merge
  * mm/vmalloc.c: preload a CPU with one object for split purpose
  * mm/vmalloc.c: remove "node" argument
  * mm/vmalloc.c: fix typo in comment
  * vmalloc: export __vmalloc_node_range for CONFIG_TEST_VMALLOC_MODULE
  * mm/vmalloc: pass VM_USERMAP flags directly to __vmalloc_node_range()
  * mm/vmalloc: do not call kmemleak_free() on not yet accounted memory
  * mm/vmalloc.c: make vmalloc_32_user() align base kernel virtual address to SHMLBA
  * mm: convert totalram_pages and totalhigh_pages variables to atomic
  * vfree: add debug might_sleep()
  * mm/vmalloc.c: improve vfree() kerneldoc
  * drivers: scsi: ufs: Remove unused d_lu_wb_buf_alloc
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64/defconfig: umi: Disable QHEE kernel memory protection
  * qcacld-3.0: Free a bunch of pkts at once
  * Makefile: Use O3 optimization level for Clang LTO
  * Makefile: Use -O3 optimization level
  * drivers: thermal: Don't qualify thermal polling as high priority
  * ARM64/defconfig: umi: Enable power efficient workqueues
  * ARM64/defconfig: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64/defconfig: Disable serial console support
  * arm64: Don't build legacy QCOM DTS.
  * rcu: fix a performance regression
  * qcacmn: Fix build error with !IPA_OFFLOAD
  * ARM64/defconfig: umi: Disable qti core control and sched autogroup
  * ARM: dts: kona: Disable IRQ debugging
  * ARM64/defconfig: umi: Disable some debug drivers
  * f2fs: Demote GC thread to idle scheduler class
  * f2fs: Set ioprio of GC kthread to idle
  * f2fs: Enlarge min_fsync_blocks to 20
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * UPSTREAM: zram: move backing_dev under macro CONFIG_ZRAM_WRITEBACK
  * UPSTREAM: zram: fix broken page writeback
  * UPSTREAM: zram: fix return value on writeback_store
  * UPSTREAM: zram: support page writeback
  * BACKPORT: zcomp: Use ARRAY_SIZE() for backends list
  * BACKPORT: zram: Allocate struct zcomp_strm as per-CPU memory
  * Revert "zram: introduce zram_entry to prepare dedup functionality"
  * Revert "zram: implement deduplication in zram"
  * Revert "zram: make deduplication feature optional"
  * Revert "zram: compare all the entries with same checksum for deduplication"
  * Revert "zram: fix race condition while returning zram_entry refcount"
  * ASoC: pcm: Add 24bit playback audio support
  * ARM64: configs: enable CONFIG_WIREGUARD
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * arm64/defconfig: umi: Enable crypto LZ4
  * arm64/defconfig: umi: Enable zram-writeback support
  * ARM64: umi/defconfigs: Bringup custom config
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
  * block: zram: Fix idle/writeback string compare
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * drivers: gpu: msm: Only build adreno 6xx part
  * video: backlight: disable modules enabled by default
  * media: gspca: disable gspca module
  * net: disable bridge netfilter module
  * net: ipv4: disable modules which are enabled by default
  * qcacmn: Fix suspicious string concatenation warning in fwlog
  * rmnet_shs: Fix CFI violation in packet assignment
  * rmnet_perf: Fix CFI violation in packet deaggregation
  * ntfs: Fix ntfs_test_inode and ntfs_init_locked_inode function type If the kernel is built with CFI we hit a __cfi_check_fail while mounting a partition
  * power: supply: Classify Battery Monitor Systems as batteries
  * cpufreq: stats: Replace the global lock with atomic.
  * techpack: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * techpack: camera: Unmap secure buffers in secure usecase
  * subsystem_restart: Always performs soft resets when subsystems crash
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * platform: msm: gsi: Export symbols only if compiled as module
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * scsi: ufs: Force enable write booster feature on UFS 3.1
  * scsi: ufs: Disable write booster feature support on UFS 2.2
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * treewide: Remove all Android.mk files
  * techpack: audio: Silence some logspam
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: Remove xiaomi early fingerprint wakeup optimization
  * audio: swr-mstr-ctrl: Fix unbalanced IRQ condition in swrm_runtime_suspend()
  * ARM64: configs: Enable XFRM_MIGRATE
  * ARM64: configs: Remove some debugging related features
  * ARM64: configs: Enable ARM64 Crypto Extensions SM3/4 and CRC32
  * ARM64: configs: Enable NTFS Filesystem
  * ARM64: configs: Enable ExFAT Filesystem
  * msm-poweroff: Store restart reason in panic
  * printk: Increase kernel log buffer size
  * smb5: Fix SDP current override for USB phy speed
  * mm: Disable watermark boost feature for K4.19
  * ARM64: configs: Skip file system sync in suspend
  * ARM64: configs: Disable PASR on kona devices
  * ARM64: configs: Reduce ION pool fill mark to 48MB
  * ARM64: configs: Build seperate DTB/DTBOs
  * ARM64: configs: Concatenate dtbs inside kernel image
  * ARM64: configs: Build Compressed Kernel Image
  * ARM64: configs: Build Qualcomm Atheros CLD WLAN module
  * ARM64: configs: Enable CPU frequency transition statistics
  * ARM64: configs: thyme: Enable support for block device writeback throttling
  * ARM64: configs: lmi: Enable AW2015 RGB LED support
  * ARM64: configs: lmi: Enable camera motor drivers
  * ARM64: configs: cas: Enable thermal dimming
  * ARM64: configs: apollo,cas: Enable panic on oops
  * ARM64: configs: apollo: Enable GPIO testing mode driver
  * ARM64: configs: Increase FRAME_WARN size to 4096
  * ARM64: configs: Enable Dynamic printk support
  * ARM64: configs: Disable MSM 11AD chip support
  * ARM64: configs: Enable xiaomi touchfeature driver
  * ARM64: configs: Disable USB CDC ADM support
  * ARM64: configs: Disable APP Armor security
  * ARM64: configs: Enable Ultrasound proximity driver
  * ARM64: configs: Enable USB Remote NDIS support
  * ARM64: configs: Disable USB Serial converter support
  * ARM64: configs: Disable V4L2 Videobuf2 Core
  * ARM64: configs: Enable SPI connected IR LED support
  * ARM64: configs: Enable PM8008 regulator driver
  * ARM64: configs: Enable device specific power supply drivers
  * ARM64: configs: Enable high performance logchar driver
  * ARM64: configs: Enable fastboot oem uart-enable command support
  * ARM64: configs: Disable support for Virtual terminal
  * ARM64: configs: Enable device specific fingerprint drivers
  * ARM64: configs: Enable AWINIC AW8697 haptics driver
  * ARM64: configs: Disable QTI Haptics support
  * ARM64: configs: Enable device specific touchscreen drivers
  * ARM64: configs: Disable Open-channel SSD target support
  * ARM64: configs: Enable CNSS QCA6390 chipset support
  * ARM64: configs: Enable QTI Smart Link Aggregation driver
  * ARM64: configs: Enable NAT, PEDIT and CSUM classifier Actions
  * ARM64: configs: Disable DEBUG_FS
  * ARM64: configs: Build Wilocity wil6210 driver
  * ARM64: configs: Enable Memory Technology Device (MTD) support
  * ARM64: configs: Disable Modules signature
  * ARM64: configs: Enable pstore support
  * ARM64: configs: Increase Kernel log buffer size
  * ARM64: configs: Enable Audit support
  * ARM64: configs: Enable device specific board platform
  * ARM64: configs: Generate xiaomi sm8250 devices configs from kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * bpf: Get the comm hash of the socket process stored inside sk buffer
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
  * fs: fuse: Implement FUSE passthrough
  * disp: msm: sde: Force SDE fd to start from 1
  * firmware: Import xiaomi touchscreens firmware
  * video: backlight: qcom-spmi-wled: Disable CABC on low brightness
  * video: backlight: Add support for thermal backlight dimming on J1S
  * video: backlight: Create brightness clone sysfs node
  * video: backlight: Notify on brightness node change
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * staging: android: ion: adjust system heap pool orders
  * staging: android: ion: Add xiaomi ion camera heap modifications
  * spi: gen-qcom: increase transfer timeout
  * soc: qcom: Add socinfo support for xiaomi sm8250 and sm7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
  * scsi: ufs: increase query timeout
  * mtd: devices: block2mtd: Sync scsi I/O on panic
  * debug: Add support to dump kmsg logs in kernel panic/oops using mtdoops
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * pinctrl: msm: Disable access to specified reserved gpios
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * cnss2: Add support for loading different bdwlan firmwares
  * net: cnss2: Add paramter to disable NV MAC support
  * net: cnss2: Increase cnss-daemon qmi timeout
  * media: rc: Add xiaomi modifications to IR driver
  * media: msm: cvp: Replace fence request handler kthread with workqueue
  * drivers: leds: Allow switching between dual flashlight leds
  * drivers: leds: Add support for AWINIC AW2015 3 Channel LED driver
  * drivers: cpuidle: Add xiaomi parameter to disable cpuidle and lpm
  * drivers: iio: Add support for Xiaomi ultrasound proximity sensor
  * drm: msm: Import xiaomi drm modifications
  * thermal: bcl_pmic5: Apply xiaomi thermal modifications
  * cpufreq: qcom: Always report maximum value of freq in limits_mitigation_notify()
  * drivers: thermal: Switch to CPUFREQ_ADJUST
  * mhi: mhi_qcom: Apply xiaomi modifications to sdx55 modem
  * drivers: misc: Import xiaomi GPIO testing mode driver
  * drivers: misc: Import Texas Instruments H-Bridge Stepper Motor Driver
  * drivers: misc: Import AKM Sensortecs AK09970 HALL sensor driver
  * block: Disable preemption before request_fn calls
  * ARM64: dts: qcom: Import kona mtp devicetree
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * input: touchscreen: Add support for xiaomi touchfeature dirver
  * input: touchscreen: Add support for Novatek NT36672c SPI touchscreen driver
  * input: touchscreen: Add support for GOODIX GT9886 touchscreen driver
  * input: touchscreen: Add support for STM FTS521 touchscreen driver
  * input: touchscreen: Add support for FocalTech ft8719/ft5452 touchscreen drivers
  * input: touchscreen: focaltech_spi: Don't ignore firmware files
  * input: touchscreen: Add support for FocalTech ft3658 touchscreen driver
  * input: touchscreen: Disable caf default touchscreen drivers
  * input: misc: aw8697_haptic: Add support to reload firmware for umi
  * input: misc: Add support for AWINIC AW8696 Haptic driver
  * input: fingerprint: Add support for GOODIX FOD fingerprint driver
  * input: fingerprint: Add support for GOODIX fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * drivers: Add support for memory fragmentation test simulator driver
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * techpack: audio: nuke xlogchar
  * Merge commit '99646a7c5d9584934073bd5c49438993f44c2f31' into android11-base
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * Add 'arch/arm64/boot/dts/vendor/' from commit 'bd2d6b0afa1f8aba19e41ce3bc29c16745595efa'
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * techpack: data: Build high performance ipa/rmnet drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * Revert "selinux: Relocate ss_initialized and selinux_enforcing to separate 4k"
  * ipa3: fix improper size checks
  * msm: ipa: Fix Makefile
  * kbuild: Remove gcc-wrapper
  * add toggle for disabling newly added USB devices
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * techpack/pinctrl-lpi: initialise at late_initcall
  * techpack: audio: makefile: do not export all the variables
  * techpack: audio: Correct symlinks
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into lineage-19.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/camera-kernel into 'LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-00300-XXXX.QSSI12.0
  * msm: adsprpc: Handle UAF in fastrpc debugfs read
  * usb: gadget: f_hid: Fix unbind issues
  * usb: gadget: f_uvc: Fix unbind issues
  * Merge "clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI"
  * msm: synx: remove synx handle details from logging
  * clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI
  * Merge "input: misc: qcom-power-on: Add support to log KPDPWR status"
  * Merge "msm: kgsl: Fix gpuaddr_in_range() to check upper bound"
  * mmc: sdhci-msm: Ensure SD card power isn't ON when card removed
  * Merge "clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL"
  * msm: kgsl: Fix gpuaddr_in_range() to check upper bound
  * Merge "usb: max-3421: Prevent corruption of freed memory"
  * clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL
  * spi: spi-msm-geni: Protect from register access in suspend state
  * spi: spi-msm-geni: Keep device to suspend if PM call fails
  * Merge "regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling"
  * Merge "seq_file: disallow extremely large seq buffer allocations"
  * regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling
  * Merge a425c3837db7a914ad21e6f5634b4c6dc7083b43 on remote branch
  * usb: gadget: f_uac1: Add support for UAC1 function
  * Merge "tzlog: update struct to get normal and fatal diag logs"
  * Merge "fs: crypto: Maintain reference count for class keys"
  * input: misc: qcom-power-on: Add support to log KPDPWR status
  * tzlog: update struct to get normal and fatal diag logs
  * firmware: qcom: Remove garbage characters from qsee log
  * firmware: qcom: add enlarged qsee log support
  * firmware: qcom: encrypted tz and qsee log support
  * fs: crypto: Maintain reference count for class keys
  * defconfig: arm: msm: Added spmi_sdam support for SDM429
  * usb: max-3421: Prevent corruption of freed memory
  * seq_file: disallow extremely large seq buffer allocations
  * pci_iomap: fix page fault issue on vmalloc with section mapping
  * Merge commit '82146398a44b8081ef002068af683b1f44f4b226' into kernel.lnx.4.19.r23-rel
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * defconfig: Enable Novatek NT36xxx Touch for tron target
  * cnss2: Update bound checks for sbl reg dumps to SRAM mem range

n.n.n / 2022-06-06
==================

  * cpuidle: lpm-levels: decrease reported deep idle latency
  * rmnet_ipa: Fix netdev watchdog triggering on suspend
  * msm: ipa: Do not acquire wakelocks
  * defconfig: Enable ExFAT
  * exfat: initialize content to avoid warn
  * BACKPORT: fs: add sync_blockdev_range()
  * BACKPORT: exfat: reduce block requests when zeroing a cluster
  * BACKPORT: exfat: introduce mount option 'sys_tz'
  * BACKPORT: exfat: fix referencing wrong parent directory information after renaming
  * BACKPORT: exfat: do not clear VolumeDirty in writeback
  * BACKPORT: exfat: allow access to paths with trailing dots
  * BACKPORT: exfat: fix missing REQ_SYNC in exfat_update_bhs()
  * BACKPORT: exfat: remove argument 'sector' from exfat_get_dentry()
  * BACKPORT: exfat: move super block magic number to magic.h
  * BACKPORT: exfat: fix i_blocks for files truncated over 4 GiB
  * BACKPORT: exfat: reuse exfat_inode_info variable instead of calling EXFAT_I()
  * BACKPORT: exfat: make exfat_find_location() static
  * BACKPORT: exfat: fix typos in comments
  * BACKPORT: exfat: simplify is_valid_cluster()
  * BACKPORT: exfat: fix incorrect loading of i_blocks for large files
  * BACKPORT: exfat: handle wrong stream entry size in exfat_readdir()
  * BACKPORT: exfat: avoid incorrectly releasing for root inode
  * BACKPORT: exfat: speed up iterate/lookup by fixing start point of traversing cluster chain
  * BACKPORT: exfat: improve write performance when dirsync enabled
  * BACKPORT: exfat: add support ioctl and FITRIM function
  * BACKPORT: exfat: introduce bitmap_lock for cluster bitmap access
  * BACKPORT: exfat: fix erroneous discard when clear cluster bit
  * BACKPORT: exfat: improve performance of exfat_free_cluster when using dirsync mount option
  * BACKPORT: exfat: fix shift-out-of-bounds in exfat_fill_super()
  * BACKPORT: exfat: Avoid allocating upcase table using kcalloc()
  * exfat: remove useless check in exfat_move_file()
  * exfat: remove 'rwoffset' in exfat_inode_info
  * exfat: replace memcpy with structure assignment
  * exfat: remove useless directory scan in exfat_add_entry()
  * exfat: eliminate dead code in exfat_find()
  * exfat: use i_blocksize() to get blocksize
  * exfat: fix misspellings using codespell tool
  * exfat: fix use of uninitialized spinlock on error path
  * exfat: fix pointer error checking
  * exfat: retain 'VolumeFlags' properly
  * exfat: optimize exfat_zeroed_cluster()
  * exfat: add error check when updating dir-entries
  * exfat: write multiple sectors at once
  * exfat: remove EXFAT_SB_DIRTY flag
  * exfat: fix name_hash computation on big endian systems
  * exfat: fix wrong size update of stream entry by typo
  * exfat: fix wrong hint_stat initialization in exfat_find_dir_entry()
  * exfat: fix overflow issue in exfat_cluster_to_sector()
  * [PATCH] fs: Reduce boilerplate in fsid handling
  * fs: exfat: Import `5.8-2arter97` and include to build
  * thaw_fingerprintd: Use strncmp() instead of memcmp()
  * power: Introduce OnePlus 3 fingerprintd thaw hack
  * drivers: android: Place a no-op PSI driver for SLMK
  * drivers: thermal: cleanup thermal implementation
  * defconfigs: Enable garbage collector for userspace wakeup sources
  * BACKPORT: soc: qcom: smp2p_sleepstate: Add support for multiple clients
  * BACKPORT: soc: qcom: smp2p_sleepstate: Fix compilation warning
  * BACKPORT: soc: qcom: smp2p: Fix possible smp2p entry double free
  * BACKPORT: soc: qcom: smp2p: Add support for suspend to disk
  * BACKPORT: soc: qcom: smp2p: Log unchanged entries
  * arch: arm64: Add GPU OC up to 900mhz
  * Revert "qmi_rmnet: Make powersave workqueue unbound and freezable"
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01500-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * version 10
  * techpack: display:  Implement doze mode
  * techpack: display: remove stock doze implementation
  * power: supply: revert caf upstream changes
  * kernel: power: Entirely checkout to msm-5.4
  * kernel: power: Checkout to msm-5.4
  * drivers: soc: qcom: Checkout to msm-5.4
  * arm64: dts: umi: optimise battery charging further
  * Revert "ARM64: dts: Disable Xiaomi FOD dimlayer impl"
  * msm: camera: Remove nonsensical register reads in cam_vfe_fe_reg_dump()
  * msm: camera: Fix memory leak in cam_res_mgr_probe()
  * msm-camera: Unmap secure buffers in secure usecase
  * mbcache: Speed up cache entry creation
  * arm64: select HAVE_MOVE_PMD for faster mremap
  * mm: speed up mremap by 20x on large regions
  * ARM64: kona/lito: Optimize FOD HBM to fix fod flashing
  * smb5: report fast charging when a proprietary charger is attached
  * Merge 5f40eefaf7c85daf81f7e228152226fe4601b1fe on remote branch
  * smp: Fix smp_call_function_single_async prototype
  * Makefile: Move -Wno-unused-but-set-variable out of GCC only block
  * techpack: display: temporary disable DC dimming when in fod_hbm mode
  * techpack: display: temporary disable DC dimming when in hbm mode
  * techpack: display: adapt exposure adjustment methods to sm8250
  * techpack: display: Let exposure adjustment switchable via sysfs
  * techpack: display: Introduce exposure adjustment driver
  * msm: kgsl: Use copy_struct_from_user() helper
  * version 9
  * defconfig: regen
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * techpack: Checkout camera kernel to MIcode psyche-r-oss
  * revert"ARM64: dts: Remove address from node-name for reserved-mem regions"
  * revert"ARM64: dts: Correct SLPI/ADSP/SPSS/CDSP memory regions on LMI/CMI/UMI"
  * Merge "msm: ADSPRPC: Update unsigned pd support on cDSP from kernel"
  * Merge "BACKPORT: dmabuf: fix use-after-free of dmabuf's file->f_inode"
  * Merge "lib: introduce copy_struct_from_user() helper"
  * Merge "msm: synx: release reference holding the lock"
  * msm: ADSPRPC: Update unsigned pd support on cDSP from kernel
  * version 8
  * ARM64: kona-v2.1: fix max frequency on thermals
  * ARM64: dts: xiaomi-sm8250-common: configure msm-thermal-simple
  * Revert "ARM64: dts: umi-sm8250: configure msm-thermal-simple"
  * arm64: dts: xiaomi: Allow big cluster to idle in USB perf mode
  * defconfigs: enable pelt halflife
  * ANDROID: sched/fair: correct pelt load information in sched-pelt.h
  * FROMLIST: sched/fair: add support to tune PELT ramp/decay timings
  * sched/fair: Fix kernel warning
  * UPSTREAM: sched: Fix out-of-bound access in uclamp
  * BACKPORT: sched/fair: Fix overutilized update in enqueue_task_fair()
  * kernel/sched: spread big prefer_idle tasks to little cores
  * UPSTREAM: sched/rt: Disable RT_RUNTIME_SHARE by default
  * sched: refine code for computing energy
  * sched/fair: refine can_migrate_boosted_task
  * sched/core: fix userspace affining threads incorrectly by task name.
  * kernel: sched: account for real time utilization
  * sched: Improve the scheduler
  * sched/core: Fix use after free issue in is_sched_lib_based_app()
  * sched/core: fix userspace affining threads incorrectly
  * sched: fine tune task placement for prioritized tasks
  * sched: fair: placement optimization for heavy load
  * Revert "sched: fine tune task placement for prioritized tasks"
  * sched: fine tune task placement for prioritized tasks
  * sched/fair: schedule lower priority tasks from little cores
  * sched/fair: do not use boosted margin for prefer_high_cap case
  * sched/fair: use actual cpu capacity to calculate boosted util
  * sched: separate capacity margin for boosted tasks
  * sched: separate boost signal from placement hint
  * kernel: sched: merge changes from LA.UM.9.12.R2.10.00.00.685.011
  * Use find_best_target to select cpu for a zero-util task
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * GKI: sched: Add back the root_domain.overutilized field
  * GKI: sched: Compile out push_task field in struct rq
  * sched: restrict iowait boost to tasks with prefer_idle
  * trace: sched: add capacity change tracing
  * sched: reduce softirq conflicts with RT
  * sched/fair: let scheduler skip util checking if cpu is idle
  * kernel: sched: Mitigate non-boosted tasks preempting boosted tasks
  * Revert "sched/core: fix userspace affining threads incorrectly"
  * Revert "sched/core: Fix use after free issue in is_sched_lib_based_app()"
  * Revert "sched: Improve the scheduler"
  * sched/fair: prefer exclusive mid cluster cpu for top-app task
  * sched: delete unused & buggy function definitions
  * sched/fair: fix implementation of is_min_capacity_cpu()
  * sched/fair: refine some scheduler changes from AU drop
  * BACKPORT: sched/fair: if sync flag ignored, try to place in mid cluster
  * Revert "sched/walt: Improve the scheduler"
  * sched: core: Disable double lock/unlock balance in move_queued_task()
  * sched: fair: Disable double lock/unlock balance in detach_task()
  * sched/fair: apply sync wake-up to pure CFS path
  * Revert "sched: fair: Always try to use energy efficient cpu for wakeups"
  * sched: fair: avoid little cpus due to sync, prev bias
  * sched: fix issue of cpu freq running at max always
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * kernel: sched: fix cpu cpu_capacity_orig being capped incorrectly
  * sched/walt: Fix negative count of sched_asym_cpucapacity static key
  * Revert "sched/fair: Add policy for restricting prefer_spread to newly idle balance"
  * fs: ext4: fsync: optimize double-fsync() a bunch
  * block : makefile : disable align mismatch
  * blk-mq: fix corruption with direct issue
  * ashmem: Adapt building on msm-4.19
  * ashmem: Rewrite to improve clarity and performance
  * block: cfq-iosched: Port samsung optimizations from SM-N986B
  * cpufreq: schedutil: Smoothen WALT predicted load boosting
  * cpufreq: schedutil: fix check for stale utilization values
  * sched: Do not reduce perceived CPU capacity while idle
  * sched/features: Fix hrtick reprogramming
  * sched: features: Disable EAS_PREFER_IDLE
  * arch: arm64: dts: qcom: kona-camera: fix i2c and order clocks correctly
  * techpack: display: Store dc status when fod hbm enabled
  * techpack: display: dsi_phy: Force allow panel phy power off
  * techpack: display: dsi_panel: Force enable Ultra-Low Power State feature for all panel
  * version 7
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * sysfs: Add sysfs_emit and sysfs_emit_at to format sysfs output
  * Revert "Add toggle for disabling newly added USB devices"
  * cpufreqX: schedutil: Use kobject release() method to free sugov_tunables
  * Merge "msm: kgsl: Remove unnecessary NULL checks"
  * Merge "msm: kgsl: Assign a default value to the variable ret"
  * dfc: bearer based QMAP powersave
  * Merge "fbdev: msm: check for valid fence before using objects"
  * Merge "sched: Improve the scheduler"
  * BACKPORT: dmabuf: fix use-after-free of dmabuf's file->f_inode
  * Merge "sched: walt: Improve the scheduler"
  * Merge 7cc7d569297d73062f58d860957f7a4edfc91033 on remote branch
  * sched: Improve the scheduler
  * lib: introduce copy_struct_from_user() helper
  * fbdev: msm: check for valid fence before using objects
  * f2fs: avoid infinite loop to flush node pages
  * f2fs: fix wrong condition check when failing metapage read
  * f2fs: remove unnecessary f2fs_lock_op in f2fs_new_inode
  * f2fs: fix to do sanity check on .cp_pack_total_block_count
  * f2fs: compress: fix to print raw data size in error path of lz4 decompression
  * f2fs: remove redundant parameter judgment
  * drivers: input: aw8697: Fix missmerge
  * ARM64: dts: Use new awinic haptics configuration
  * input: misc: aw8697_haptic: Upgrade from zeus-s-oss
  * arch: dts: Throttle charging speed and Optimise charging current
  * msm: thermal: simple: account for battery temperature in calculations
  * arm64: lib: Memory utilities optimization
  * proc: do not let google camera run in the background
  * Merge "diag: Use correct size while reallocating for hdlc encoding"
  * Merge "qseecom : qseecom_scale_bus_bandwidth doesn't check the negative mode"
  * Merge "soc: qcom: mdt_loader: Replacing sprintf with snprintf"
  * Merge "defconfig: arm: msm: Added spmi_sdam support for SDM429"
  * msm:ipa3: Fixed pointer dereference issue without checking for null
  * msm: kgsl: Assign a default value to the variable ret
  * msm: kgsl: Remove unnecessary NULL checks
  * diag: Use correct size while reallocating for hdlc encoding
  * Merge "tasks, sched/core: Ensure tasks are available for a grace period after leaving the runqueue"
  * msm: synx: release reference holding the lock
  * sched: walt: Improve the scheduler
  * Merge "defconfig: Enable UAC1 config for kona"
  * tasks, sched/core: Ensure tasks are available for a grace period after leaving the runqueue
  * tasks: Add a count of task RCU users
  * Merge "usb: misc: Add snapshot of diag_ipc_bridge driver"
  * Merge "usb: misc: ks_bridge: Add snapshot of ks_bridge driver"
  * usb: misc: Add snapshot of diag_ipc_bridge driver
  * Merge "defconfig: msm: Enable LED for QCS2290"
  * usb: misc: ks_bridge: Add snapshot of ks_bridge driver
  * Merge "msm: adsprpc: Wait for actual shutdown to complete"
  * Merge "coresight: Replacing sprintf with scnprintf"
  * msm: adsprpc: Wait for actual shutdown to complete
  * defconfig: msm: Enable LED for QCS2290
  * qseecom : qseecom_scale_bus_bandwidth doesn't check the negative mode
  * Merge "defconfig: kona: Enable mcp25xxfd driver for perf_defconfig"
  * coresight: Replacing sprintf with scnprintf
  * Merge "drivers: can: Enclose mcp25xxfd_dump_regs into CONFIG_DEBUG_FS"
  * defconfig: kona: Enable mcp25xxfd driver for perf_defconfig
  * drivers: can: Enclose mcp25xxfd_dump_regs into CONFIG_DEBUG_FS
  * msm: adsprpc:  Fix double fetch from fastrpc HLOS driver
  * soc: qcom: mdt_loader: Replacing sprintf with snprintf
  * Merge 7e6242a76c9ada28629e35bc29bdc8c9950c1b59 on remote branch
  * Merge "msm: adsprpc: Do length check to avoid arbitrary memory access"
  * msm: adsprpc: Do length check to avoid arbitrary memory access
  * Merge "pci_iomap: fix page fault issue on vmalloc with section mapping"
  * Merge "aio: fix use-after-free due to missing POLLFREE handling"
  * Merge "FROMGIT: USB: gadget: bRequestType is a bitfield, not a enum"
  * Merge "aio: keep poll requests on waitqueue until completed"
  * Merge "sctp: add param size validation for SCTP_PARAM_SET_PRIMARY"
  * Merge "binder: use wake_up_pollfree()"
  * Merge "fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs"
  * thermal: Increase thermal trip points to 16
  * f2fs: use spin_lock to avoid hang
  * f2fs: remove unnecessary read for F2FS_FITS_IN_INODE
  * f2fs: avoid an infinite loop in f2fs_sync_dirty_inodes
  * f2fs: fix to do sanity check on curseg->alloc_type
  * f2fs: fix to avoid potential deadlock
  * f2fs: fix missing free nid in f2fs_handle_failed_inode
  * f2fs: add a way to limit roll forward recovery time
  * f2fs: adjust readahead block number during recovery
  * f2fs: expose discard related parameters in sysfs
  * f2fs: move discard parameters into discard_cmd_control
  * f2fs: do not allow partial truncation on pinned file
  * icnss: Add code to pass device configs to wlan driver
  * fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs
  * msm: kgsl: Zap performance counters across context switches
  * msm: kgsl: Add a sysfs node to control performance counter reads
  * disp: uapi: drm: Add new blend type for CAC
  * Merge "msm: npu: Avoid buffer overflow when handling get_property packet"
  * Merge "USB: uac1: Fix audio quality issues for UAC1"
  * Merge "msm_bus: Check if cldata->pdata is null before access it"
  * version 6
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * USB: uac1: Fix audio quality issues for UAC1
  * defconfig: Enable UAC1 config for kona
  * UAC1: Add configfs attribute for changing ep max packet size
  * USB: f_uac1: Add check before accessing members of uac1
  * USB: f_uac1: add iad descriptor for UAC1 driver
  * f_uac1: Remove unwanted descriptors for volume/mute support
  * f_uac1: Add support for volume/mute control settings
  * msm: npu: Avoid buffer overflow when handling get_property packet
  * sctp: add param size validation for SCTP_PARAM_SET_PRIMARY
  * sctp: validate chunk size in __rcv_asconf_lookup
  * sctp: add size validation when walking chunks
  * sctp: validate from_addr_param return
  * aio: fix use-after-free due to missing POLLFREE handling
  * aio: keep poll requests on waitqueue until completed
  * signalfd: use wake_up_pollfree()
  * binder: use wake_up_pollfree()
  * wait: add wake_up_pollfree()
  * FROMGIT: USB: gadget: bRequestType is a bitfield, not a enum
  * UPSTREAM: USB: gadget: zero allocate endpoint 0 buffers
  * UPSTREAM: USB: gadget: detect too-big endpoint 0 requests
  * Merge "cnss2: Handle event processing after shutdown"
  * Merge "soc: qcom: socinfo: Add soc information for KhajeP and KhajeQ"
  * Merge "i2c: i2c-msm-v2: Set frequency parameters to INT and validate"
  * Merge d09499e9c8a99763bf9afa92c06601070523dc3e on remote branch
  * Merge "drivers: thermal: validate cdev sysfs state request before using it"
  * soc: qcom: socinfo: Add soc information for KhajeP and KhajeQ
  * msm_bus: Check if cldata->pdata is null before access it
  * spi: spi-msm-geni: Put device to suspend if PM status is active
  * Merge "cfg80211: Add support to advertize OCV support"
  * Merge "spi: spi-msm-geni: Protect from register access in suspend state"
  * Merge "spi: spi-msm-geni: Keep device to suspend if PM call fails"
  * cnss2: Handle event processing after shutdown
  * cfg80211: Add support to advertize OCV support
  * i2c: i2c-msm-v2: Set frequency parameters to INT and validate
  * msm: kgsl: Reduce latency while processing ioctls
  * setlocalversion: Never append a "+" to localversion
  * drivers:block:zram:default lz4
  * zram: Move default compression algorithm choice to Kconfig
  * defconfig: enable lz4 compression
  * drivers: thermal: validate cdev sysfs state request before using it
  * defconfigs: disable msm performance
  * USB: f_fs: Fix disconnect check during ongoing IO
  * Merge "msm: diag: fix copyright"
  * Merge "drivers: soc: qcom: Initialize blocking notifier as per lockdep"
  * msm: diag: fix copyright
  * Merge "diag: Validate the dci client before sending dci packet"
  * diag: Validate the dci client before sending dci packet
  * drivers: soc: qcom: Initialize blocking notifier as per lockdep
  * version 5
  * defconfig: regen
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * usb: gadget: u_audio: fix race condition on endpoint stop
  * usb: gadget: u_audio: Free requests only after callback
  * defconfig: Enable usb peripheral audio on SXR2130
  * Merge "diag: Ensure dci entry is valid before sending the packet"
  * Merge "msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()"
  * msm: ADSPRPC: Fix to avoid Use after free in fastrpc_init_process
  * Merge "msm: synx: fix copyright"
  * Merge "msm: ipa3: Fix to validate the NAT table entries during NAT table init"
  * msm: synx: fix copyright
  * clk: qcom: vdd-level: Update the vdd level for CX on Khaje
  * msm: ipa3: Fix to validate the NAT table entries during NAT table init
  * Merge 533e678218e41df69501158ff6eb84deafb6771c on remote branch
  * msm: synx: fix copyright
  * clk: qcom: gpucc: Update the vdd level for CX on Khaje
  * diag: Ensure dci entry is valid before sending the packet
  * Merge "usb: pd: Process request message as soon as it is received"
  * usb: pd: Process request message as soon as it is received
  * Merge "dfc: reset tx queue"
  * mmc: sdhci-msm: Update the MGPI SW check to avoid irq timeout
  * msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()
  * Merge "msm: ipa3: fix to cleanup the dma allocation"
  * Merge "msm: synx: remove synx handle details from logging"
  * dfc: reset tx queue
  * clk: qcom: gpucc: Add support for higher frequency for Khaje
  * Merge "clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje"
  * msm: ipa3: fix to cleanup the dma allocation
  * Merge "include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner"
  * clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje
  * Merge "msm: adsprpc: Handle UAF in fastrpc debugfs read"
  * include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner
  * Merge "usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind"
  * usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind
  * version 4
  * firmware: focaltech: Upgrade FW from haydin
  * touchscreen: focaltech_spi: Upgrade regardless of FW version check
  * techpack: audio: Mark msm-dai-q6 drivers as sync probe
  * techpack/audio: kona: Report correct key code of headset buttons
  * thermal: qcom: bcl: Return before thermal trip evaluates
  * defconfigs: enable Simple MSM Thermal
  * ARM64: dts: umi-sm8250: configure msm-thermal-simple
  * msm: thermal: simple: Introduce simple MSM thermal solution [msm-4.14]
  * version 3
  * drivers: thermal: limits-dcvs: Always build driver
  * drivers: power: supply: Disable battery capacity learning
  * defconfig: revert back to 100Hz tick
  * build: version 2
  * build: its Snow time !!!
  * drivers: usb: dwc3: Specify sync probe for dwc3-of-simple.
  * drivers: usb: dwc3: Specify sync probe for msm-dwc3 driver
  * drivers: usb: dwc3: Specify sync probe for dwc3 driver
  * drivers: usb: pd: remove fix pd output for 5v
  * Revert "usb: pd: Register typec partner in case AAA is connected"
  * PM / sleep: Skip OOM killer toggles when kernel is compiled for Android
  * drm: msm: optimize interpolation
  * techpack: drm: sde: fix a race condition
  * platform: msm: Fix dangerous relocation
  * defconfig: Bump SLMK minfree & timeout
  * power: process: Use lesser time to enter sleep
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * qos: Don't allow userspace to impose restrictions on CPU idle levels
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * dts: kona-gpu: Remove qos active latency node
  * treewide: use mi drm notifier
  * drivers: power: add timeouts to wakelocks
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * ARM64: umi: overlay gpu data from kona-v2-gpu.dtsi
  * arm64: dts: kona-v2-gpu: Overclock to 670mhz
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * gitignore: add out directory
  * add yarpiins build stuff
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/defconfig: apollo: Bringup custom defconfig
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * Revert "scsi: ufs: increase query timeout"
  * Revert "drivers: Add support for memory fragmentation test simulator driver"
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * ANDROID: sched: Exempt paused CPU from nohz idle balance
  * ARM64: configs: Make configs required to pass CTS tests mandatory
  * ARM64/defconfig: umi: Disable LSE atomics
  * ARM64/defconfig: umi: Use a timer frequency of 100 Hz
  * arm64: Inline the spin lock function family
  * ARM64/defconfig: umi: Disable unused errata
  * ARM64/defconfig: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * UPSTREAM: arm64: link with -z norelro for LLD or aarch64-elf
  * techpack: display: msm: dsi: set doze brightness for aod correctly
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64/defconfig: umi: Disable some unuse drivers
  * ARM64/defconfig: umi: Disable stability debug configs Disable the following stability debug configs for shipping: - CONFIG_EDAC_KRYO_ARM64_PANIC_ON_UE
  * ARM64/defconfig: umi: Enable LLD, RELR, Clang ThinLTO optimizations
  * ANDROID: sched: EAS: take cstate into account when selecting idle core
  * ARM64/defconfig: umi: Enable jump label
  * arm64/kernel: jump_label: Switch to relative references
  * locking/static_key: Add support for deferred static branches
  * jump_label: Fix NULL dereference bug in __jump_label_mod_update()
  * jump_label: Annotate entries that operate on __init code earlier
  * jump_label: Implement generic support for relative references
  * jump_label: Abstract jump_entry member accessors
  * jump_label: Use static_key_linked() accessor
  * mm/vmalloc.c: switch to WARN_ON() and move it under unlink_va()
  * mm/vmalloc.c: get rid of one single unlink_va() when merge
  * mm/vmalloc.c: preload a CPU with one object for split purpose
  * mm/vmalloc.c: remove "node" argument
  * mm/vmalloc.c: fix typo in comment
  * vmalloc: export __vmalloc_node_range for CONFIG_TEST_VMALLOC_MODULE
  * mm/vmalloc: pass VM_USERMAP flags directly to __vmalloc_node_range()
  * mm/vmalloc: do not call kmemleak_free() on not yet accounted memory
  * mm/vmalloc.c: make vmalloc_32_user() align base kernel virtual address to SHMLBA
  * mm: convert totalram_pages and totalhigh_pages variables to atomic
  * vfree: add debug might_sleep()
  * mm/vmalloc.c: improve vfree() kerneldoc
  * drivers: scsi: ufs: Remove unused d_lu_wb_buf_alloc
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64/defconfig: umi: Disable QHEE kernel memory protection
  * qcacld-3.0: Free a bunch of pkts at once
  * Makefile: Use O3 optimization level for Clang LTO
  * Makefile: Use -O3 optimization level
  * drivers: thermal: Don't qualify thermal polling as high priority
  * ARM64/defconfig: umi: Enable power efficient workqueues
  * ARM64/defconfig: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64/defconfig: Disable serial console support
  * arm64: Don't build legacy QCOM DTS.
  * rcu: fix a performance regression
  * qcacmn: Fix build error with !IPA_OFFLOAD
  * ARM64/defconfig: umi: Disable qti core control and sched autogroup
  * ARM: dts: kona: Disable IRQ debugging
  * ARM64/defconfig: umi: Disable some debug drivers
  * f2fs: Demote GC thread to idle scheduler class
  * f2fs: Set ioprio of GC kthread to idle
  * f2fs: Enlarge min_fsync_blocks to 20
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * UPSTREAM: zram: move backing_dev under macro CONFIG_ZRAM_WRITEBACK
  * UPSTREAM: zram: fix broken page writeback
  * UPSTREAM: zram: fix return value on writeback_store
  * UPSTREAM: zram: support page writeback
  * BACKPORT: zcomp: Use ARRAY_SIZE() for backends list
  * BACKPORT: zram: Allocate struct zcomp_strm as per-CPU memory
  * Revert "zram: introduce zram_entry to prepare dedup functionality"
  * Revert "zram: implement deduplication in zram"
  * Revert "zram: make deduplication feature optional"
  * Revert "zram: compare all the entries with same checksum for deduplication"
  * Revert "zram: fix race condition while returning zram_entry refcount"
  * ASoC: pcm: Add 24bit playback audio support
  * ARM64: configs: enable CONFIG_WIREGUARD
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * arm64/defconfig: umi: Enable crypto LZ4
  * arm64/defconfig: umi: Enable zram-writeback support
  * ARM64: umi/defconfigs: Bringup custom config
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
  * block: zram: Fix idle/writeback string compare
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * drivers: gpu: msm: Only build adreno 6xx part
  * video: backlight: disable modules enabled by default
  * media: gspca: disable gspca module
  * net: disable bridge netfilter module
  * net: ipv4: disable modules which are enabled by default
  * qcacmn: Fix suspicious string concatenation warning in fwlog
  * rmnet_shs: Fix CFI violation in packet assignment
  * rmnet_perf: Fix CFI violation in packet deaggregation
  * ntfs: Fix ntfs_test_inode and ntfs_init_locked_inode function type If the kernel is built with CFI we hit a __cfi_check_fail while mounting a partition
  * power: supply: Classify Battery Monitor Systems as batteries
  * cpufreq: stats: Replace the global lock with atomic.
  * techpack: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * techpack: camera: Unmap secure buffers in secure usecase
  * subsystem_restart: Always performs soft resets when subsystems crash
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * platform: msm: gsi: Export symbols only if compiled as module
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * scsi: ufs: Force enable write booster feature on UFS 3.1
  * scsi: ufs: Disable write booster feature support on UFS 2.2
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * treewide: Remove all Android.mk files
  * techpack: audio: Silence some logspam
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: Remove xiaomi early fingerprint wakeup optimization
  * audio: swr-mstr-ctrl: Fix unbalanced IRQ condition in swrm_runtime_suspend()
  * ARM64: configs: Enable XFRM_MIGRATE
  * ARM64: configs: Remove some debugging related features
  * ARM64: configs: Enable ARM64 Crypto Extensions SM3/4 and CRC32
  * ARM64: configs: Enable NTFS Filesystem
  * ARM64: configs: Enable ExFAT Filesystem
  * msm-poweroff: Store restart reason in panic
  * printk: Increase kernel log buffer size
  * smb5: Fix SDP current override for USB phy speed
  * mm: Disable watermark boost feature for K4.19
  * ARM64: configs: Skip file system sync in suspend
  * ARM64: configs: Disable PASR on kona devices
  * ARM64: configs: Reduce ION pool fill mark to 48MB
  * ARM64: configs: Build seperate DTB/DTBOs
  * ARM64: configs: Concatenate dtbs inside kernel image
  * ARM64: configs: Build Compressed Kernel Image
  * ARM64: configs: Build Qualcomm Atheros CLD WLAN module
  * ARM64: configs: Enable CPU frequency transition statistics
  * ARM64: configs: thyme: Enable support for block device writeback throttling
  * ARM64: configs: lmi: Enable AW2015 RGB LED support
  * ARM64: configs: lmi: Enable camera motor drivers
  * ARM64: configs: cas: Enable thermal dimming
  * ARM64: configs: apollo,cas: Enable panic on oops
  * ARM64: configs: apollo: Enable GPIO testing mode driver
  * ARM64: configs: Increase FRAME_WARN size to 4096
  * ARM64: configs: Enable Dynamic printk support
  * ARM64: configs: Disable MSM 11AD chip support
  * ARM64: configs: Enable xiaomi touchfeature driver
  * ARM64: configs: Disable USB CDC ADM support
  * ARM64: configs: Disable APP Armor security
  * ARM64: configs: Enable Ultrasound proximity driver
  * ARM64: configs: Enable USB Remote NDIS support
  * ARM64: configs: Disable USB Serial converter support
  * ARM64: configs: Disable V4L2 Videobuf2 Core
  * ARM64: configs: Enable SPI connected IR LED support
  * ARM64: configs: Enable PM8008 regulator driver
  * ARM64: configs: Enable device specific power supply drivers
  * ARM64: configs: Enable high performance logchar driver
  * ARM64: configs: Enable fastboot oem uart-enable command support
  * ARM64: configs: Disable support for Virtual terminal
  * ARM64: configs: Enable device specific fingerprint drivers
  * ARM64: configs: Enable AWINIC AW8697 haptics driver
  * ARM64: configs: Disable QTI Haptics support
  * ARM64: configs: Enable device specific touchscreen drivers
  * ARM64: configs: Disable Open-channel SSD target support
  * ARM64: configs: Enable CNSS QCA6390 chipset support
  * ARM64: configs: Enable QTI Smart Link Aggregation driver
  * ARM64: configs: Enable NAT, PEDIT and CSUM classifier Actions
  * ARM64: configs: Disable DEBUG_FS
  * ARM64: configs: Build Wilocity wil6210 driver
  * ARM64: configs: Enable Memory Technology Device (MTD) support
  * ARM64: configs: Disable Modules signature
  * ARM64: configs: Enable pstore support
  * ARM64: configs: Increase Kernel log buffer size
  * ARM64: configs: Enable Audit support
  * ARM64: configs: Enable device specific board platform
  * ARM64: configs: Generate xiaomi sm8250 devices configs from kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * bpf: Get the comm hash of the socket process stored inside sk buffer
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
  * fs: fuse: Implement FUSE passthrough
  * disp: msm: sde: Force SDE fd to start from 1
  * firmware: Import xiaomi touchscreens firmware
  * video: backlight: qcom-spmi-wled: Disable CABC on low brightness
  * video: backlight: Add support for thermal backlight dimming on J1S
  * video: backlight: Create brightness clone sysfs node
  * video: backlight: Notify on brightness node change
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * staging: android: ion: adjust system heap pool orders
  * staging: android: ion: Add xiaomi ion camera heap modifications
  * spi: gen-qcom: increase transfer timeout
  * soc: qcom: Add socinfo support for xiaomi sm8250 and sm7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
  * scsi: ufs: increase query timeout
  * mtd: devices: block2mtd: Sync scsi I/O on panic
  * debug: Add support to dump kmsg logs in kernel panic/oops using mtdoops
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * pinctrl: msm: Disable access to specified reserved gpios
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * cnss2: Add support for loading different bdwlan firmwares
  * net: cnss2: Add paramter to disable NV MAC support
  * net: cnss2: Increase cnss-daemon qmi timeout
  * media: rc: Add xiaomi modifications to IR driver
  * media: msm: cvp: Replace fence request handler kthread with workqueue
  * drivers: leds: Allow switching between dual flashlight leds
  * drivers: leds: Add support for AWINIC AW2015 3 Channel LED driver
  * drivers: cpuidle: Add xiaomi parameter to disable cpuidle and lpm
  * drivers: iio: Add support for Xiaomi ultrasound proximity sensor
  * drm: msm: Import xiaomi drm modifications
  * thermal: bcl_pmic5: Apply xiaomi thermal modifications
  * cpufreq: qcom: Always report maximum value of freq in limits_mitigation_notify()
  * drivers: thermal: Switch to CPUFREQ_ADJUST
  * mhi: mhi_qcom: Apply xiaomi modifications to sdx55 modem
  * drivers: misc: Import xiaomi GPIO testing mode driver
  * drivers: misc: Import Texas Instruments H-Bridge Stepper Motor Driver
  * drivers: misc: Import AKM Sensortecs AK09970 HALL sensor driver
  * block: Disable preemption before request_fn calls
  * ARM64: dts: qcom: Import kona mtp devicetree
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * input: touchscreen: Add support for xiaomi touchfeature dirver
  * input: touchscreen: Add support for Novatek NT36672c SPI touchscreen driver
  * input: touchscreen: Add support for GOODIX GT9886 touchscreen driver
  * input: touchscreen: Add support for STM FTS521 touchscreen driver
  * input: touchscreen: Add support for FocalTech ft8719/ft5452 touchscreen drivers
  * input: touchscreen: focaltech_spi: Don't ignore firmware files
  * input: touchscreen: Add support for FocalTech ft3658 touchscreen driver
  * input: touchscreen: Disable caf default touchscreen drivers
  * input: misc: aw8697_haptic: Add support to reload firmware for umi
  * input: misc: Add support for AWINIC AW8696 Haptic driver
  * input: fingerprint: Add support for GOODIX FOD fingerprint driver
  * input: fingerprint: Add support for GOODIX fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * drivers: Add support for memory fragmentation test simulator driver
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * techpack: audio: nuke xlogchar
  * Merge commit '99646a7c5d9584934073bd5c49438993f44c2f31' into android11-base
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * Add 'arch/arm64/boot/dts/vendor/' from commit 'bd2d6b0afa1f8aba19e41ce3bc29c16745595efa'
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * techpack: data: Build high performance ipa/rmnet drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * Revert "selinux: Relocate ss_initialized and selinux_enforcing to separate 4k"
  * ipa3: fix improper size checks
  * msm: ipa: Fix Makefile
  * kbuild: Remove gcc-wrapper
  * add toggle for disabling newly added USB devices
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * techpack/pinctrl-lpi: initialise at late_initcall
  * techpack: audio: makefile: do not export all the variables
  * techpack: audio: Correct symlinks
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into lineage-19.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/camera-kernel into 'LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-00300-XXXX.QSSI12.0
  * msm: adsprpc: Handle UAF in fastrpc debugfs read
  * usb: gadget: f_hid: Fix unbind issues
  * usb: gadget: f_uvc: Fix unbind issues
  * Merge "clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI"
  * msm: synx: remove synx handle details from logging
  * clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI
  * Merge "input: misc: qcom-power-on: Add support to log KPDPWR status"
  * Merge "msm: kgsl: Fix gpuaddr_in_range() to check upper bound"
  * mmc: sdhci-msm: Ensure SD card power isn't ON when card removed
  * Merge "clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL"
  * msm: kgsl: Fix gpuaddr_in_range() to check upper bound
  * Merge "usb: max-3421: Prevent corruption of freed memory"
  * clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL
  * spi: spi-msm-geni: Protect from register access in suspend state
  * spi: spi-msm-geni: Keep device to suspend if PM call fails
  * Merge "regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling"
  * Merge "seq_file: disallow extremely large seq buffer allocations"
  * regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling
  * Merge a425c3837db7a914ad21e6f5634b4c6dc7083b43 on remote branch
  * usb: gadget: f_uac1: Add support for UAC1 function
  * Merge "tzlog: update struct to get normal and fatal diag logs"
  * Merge "fs: crypto: Maintain reference count for class keys"
  * input: misc: qcom-power-on: Add support to log KPDPWR status
  * tzlog: update struct to get normal and fatal diag logs
  * firmware: qcom: Remove garbage characters from qsee log
  * firmware: qcom: add enlarged qsee log support
  * firmware: qcom: encrypted tz and qsee log support
  * fs: crypto: Maintain reference count for class keys
  * defconfig: arm: msm: Added spmi_sdam support for SDM429
  * usb: max-3421: Prevent corruption of freed memory
  * seq_file: disallow extremely large seq buffer allocations
  * pci_iomap: fix page fault issue on vmalloc with section mapping
  * Merge commit '82146398a44b8081ef002068af683b1f44f4b226' into kernel.lnx.4.19.r23-rel
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * defconfig: Enable Novatek NT36xxx Touch for tron target
  * cnss2: Update bound checks for sbl reg dumps to SRAM mem range

n.n.n / 2022-05-29
==================

  * techpack: display:  Implement doze mode
  * techpack: display: remove stock doze implementation
  * power: supply: revert caf upstream changes
  * kernel: power: Entirely checkout to msm-5.4
  * kernel: power: Checkout to msm-5.4
  * drivers: soc: qcom: Checkout to msm-5.4
  * arm64: dts: umi: optimise battery charging further
  * Revert "ARM64: dts: Disable Xiaomi FOD dimlayer impl"
  * msm: camera: Remove nonsensical register reads in cam_vfe_fe_reg_dump()
  * msm: camera: Fix memory leak in cam_res_mgr_probe()
  * msm-camera: Unmap secure buffers in secure usecase
  * mbcache: Speed up cache entry creation
  * arm64: select HAVE_MOVE_PMD for faster mremap
  * mm: speed up mremap by 20x on large regions
  * ARM64: kona/lito: Optimize FOD HBM to fix fod flashing
  * smb5: report fast charging when a proprietary charger is attached
  * techpack: display: temporary disable DC dimming when in fod_hbm mode
  * techpack: display: temporary disable DC dimming when in hbm mode
  * techpack: display: adapt exposure adjustment methods to sm8250
  * techpack: display: Let exposure adjustment switchable via sysfs
  * techpack: display: Introduce exposure adjustment driver

n.n.n / 2022-05-03
==================

  * defconfig: regen
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-01300-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * techpack: Checkout camera kernel to MIcode psyche-r-oss
  * revert"ARM64: dts: Remove address from node-name for reserved-mem regions"
  * revert"ARM64: dts: Correct SLPI/ADSP/SPSS/CDSP memory regions on LMI/CMI/UMI"
  * version 8
  * ARM64: kona-v2.1: fix max frequency on thermals
  * ARM64: dts: xiaomi-sm8250-common: configure msm-thermal-simple
  * Revert "ARM64: dts: umi-sm8250: configure msm-thermal-simple"
  * arm64: dts: xiaomi: Allow big cluster to idle in USB perf mode
  * defconfigs: enable pelt halflife
  * ANDROID: sched/fair: correct pelt load information in sched-pelt.h
  * FROMLIST: sched/fair: add support to tune PELT ramp/decay timings
  * sched/fair: Fix kernel warning
  * UPSTREAM: sched: Fix out-of-bound access in uclamp
  * BACKPORT: sched/fair: Fix overutilized update in enqueue_task_fair()
  * kernel/sched: spread big prefer_idle tasks to little cores
  * UPSTREAM: sched/rt: Disable RT_RUNTIME_SHARE by default
  * sched: refine code for computing energy
  * sched/fair: refine can_migrate_boosted_task
  * sched/core: fix userspace affining threads incorrectly by task name.
  * kernel: sched: account for real time utilization
  * sched: Improve the scheduler
  * sched/core: Fix use after free issue in is_sched_lib_based_app()
  * sched/core: fix userspace affining threads incorrectly
  * sched: fine tune task placement for prioritized tasks
  * sched: fair: placement optimization for heavy load
  * Revert "sched: fine tune task placement for prioritized tasks"
  * sched: fine tune task placement for prioritized tasks
  * sched/fair: schedule lower priority tasks from little cores
  * sched/fair: do not use boosted margin for prefer_high_cap case
  * sched/fair: use actual cpu capacity to calculate boosted util
  * sched: separate capacity margin for boosted tasks
  * sched: separate boost signal from placement hint
  * kernel: sched: merge changes from LA.UM.9.12.R2.10.00.00.685.011
  * Use find_best_target to select cpu for a zero-util task
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * GKI: sched: Add back the root_domain.overutilized field
  * GKI: sched: Compile out push_task field in struct rq
  * sched: restrict iowait boost to tasks with prefer_idle
  * trace: sched: add capacity change tracing
  * sched: reduce softirq conflicts with RT
  * sched/fair: let scheduler skip util checking if cpu is idle
  * kernel: sched: Mitigate non-boosted tasks preempting boosted tasks
  * Revert "sched/core: fix userspace affining threads incorrectly"
  * Revert "sched/core: Fix use after free issue in is_sched_lib_based_app()"
  * Revert "sched: Improve the scheduler"
  * sched/fair: prefer exclusive mid cluster cpu for top-app task
  * sched: delete unused & buggy function definitions
  * sched/fair: fix implementation of is_min_capacity_cpu()
  * sched/fair: refine some scheduler changes from AU drop
  * BACKPORT: sched/fair: if sync flag ignored, try to place in mid cluster
  * Revert "sched/walt: Improve the scheduler"
  * sched: core: Disable double lock/unlock balance in move_queued_task()
  * sched: fair: Disable double lock/unlock balance in detach_task()
  * sched/fair: apply sync wake-up to pure CFS path
  * Revert "sched: fair: Always try to use energy efficient cpu for wakeups"
  * sched: fair: avoid little cpus due to sync, prev bias
  * sched: fix issue of cpu freq running at max always
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * kernel: sched: fix cpu cpu_capacity_orig being capped incorrectly
  * sched/walt: Fix negative count of sched_asym_cpucapacity static key
  * Revert "sched/fair: Add policy for restricting prefer_spread to newly idle balance"
  * fs: ext4: fsync: optimize double-fsync() a bunch
  * block : makefile : disable align mismatch
  * blk-mq: fix corruption with direct issue
  * ashmem: Adapt building on msm-4.19
  * ashmem: Rewrite to improve clarity and performance
  * block: cfq-iosched: Port samsung optimizations from SM-N986B
  * cpufreq: schedutil: Smoothen WALT predicted load boosting
  * cpufreq: schedutil: fix check for stale utilization values
  * sched: Do not reduce perceived CPU capacity while idle
  * sched/features: Fix hrtick reprogramming
  * sched: features: Disable EAS_PREFER_IDLE
  * arch: arm64: dts: qcom: kona-camera: fix i2c and order clocks correctly
  * techpack: display: Store dc status when fod hbm enabled
  * techpack: display: dsi_phy: Force allow panel phy power off
  * techpack: display: dsi_panel: Force enable Ultra-Low Power State feature for all panel
  * version 7
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * sysfs: Add sysfs_emit and sysfs_emit_at to format sysfs output
  * Revert "Add toggle for disabling newly added USB devices"
  * cpufreqX: schedutil: Use kobject release() method to free sugov_tunables
  * Merge 7cc7d569297d73062f58d860957f7a4edfc91033 on remote branch
  * f2fs: avoid infinite loop to flush node pages
  * f2fs: fix wrong condition check when failing metapage read
  * f2fs: remove unnecessary f2fs_lock_op in f2fs_new_inode
  * f2fs: fix to do sanity check on .cp_pack_total_block_count
  * f2fs: compress: fix to print raw data size in error path of lz4 decompression
  * f2fs: remove redundant parameter judgment
  * drivers: input: aw8697: Fix missmerge
  * ARM64: dts: Use new awinic haptics configuration
  * input: misc: aw8697_haptic: Upgrade from zeus-s-oss
  * arch: dts: Throttle charging speed and Optimise charging current
  * msm: thermal: simple: account for battery temperature in calculations
  * arm64: lib: Memory utilities optimization
  * proc: do not let google camera run in the background
  * Merge "diag: Use correct size while reallocating for hdlc encoding"
  * Merge "qseecom : qseecom_scale_bus_bandwidth doesn't check the negative mode"
  * Merge "soc: qcom: mdt_loader: Replacing sprintf with snprintf"
  * Merge "defconfig: arm: msm: Added spmi_sdam support for SDM429"
  * msm:ipa3: Fixed pointer dereference issue without checking for null
  * diag: Use correct size while reallocating for hdlc encoding
  * Merge "tasks, sched/core: Ensure tasks are available for a grace period after leaving the runqueue"
  * Merge "defconfig: Enable UAC1 config for kona"
  * tasks, sched/core: Ensure tasks are available for a grace period after leaving the runqueue
  * tasks: Add a count of task RCU users
  * Merge "usb: misc: Add snapshot of diag_ipc_bridge driver"
  * Merge "usb: misc: ks_bridge: Add snapshot of ks_bridge driver"
  * usb: misc: Add snapshot of diag_ipc_bridge driver
  * Merge "defconfig: msm: Enable LED for QCS2290"
  * usb: misc: ks_bridge: Add snapshot of ks_bridge driver
  * Merge "msm: adsprpc: Wait for actual shutdown to complete"
  * Merge "coresight: Replacing sprintf with scnprintf"
  * msm: adsprpc: Wait for actual shutdown to complete
  * defconfig: msm: Enable LED for QCS2290
  * qseecom : qseecom_scale_bus_bandwidth doesn't check the negative mode
  * Merge "defconfig: kona: Enable mcp25xxfd driver for perf_defconfig"
  * coresight: Replacing sprintf with scnprintf
  * Merge "drivers: can: Enclose mcp25xxfd_dump_regs into CONFIG_DEBUG_FS"
  * defconfig: kona: Enable mcp25xxfd driver for perf_defconfig
  * drivers: can: Enclose mcp25xxfd_dump_regs into CONFIG_DEBUG_FS
  * msm: adsprpc:  Fix double fetch from fastrpc HLOS driver
  * soc: qcom: mdt_loader: Replacing sprintf with snprintf
  * Merge 7e6242a76c9ada28629e35bc29bdc8c9950c1b59 on remote branch
  * Merge "msm: adsprpc: Do length check to avoid arbitrary memory access"
  * msm: adsprpc: Do length check to avoid arbitrary memory access
  * Merge "pci_iomap: fix page fault issue on vmalloc with section mapping"
  * Merge "aio: fix use-after-free due to missing POLLFREE handling"
  * Merge "FROMGIT: USB: gadget: bRequestType is a bitfield, not a enum"
  * Merge "aio: keep poll requests on waitqueue until completed"
  * Merge "sctp: add param size validation for SCTP_PARAM_SET_PRIMARY"
  * Merge "binder: use wake_up_pollfree()"
  * Merge "fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs"
  * thermal: Increase thermal trip points to 16
  * f2fs: use spin_lock to avoid hang
  * f2fs: remove unnecessary read for F2FS_FITS_IN_INODE
  * f2fs: avoid an infinite loop in f2fs_sync_dirty_inodes
  * f2fs: fix to do sanity check on curseg->alloc_type
  * f2fs: fix to avoid potential deadlock
  * f2fs: fix missing free nid in f2fs_handle_failed_inode
  * f2fs: add a way to limit roll forward recovery time
  * f2fs: adjust readahead block number during recovery
  * f2fs: expose discard related parameters in sysfs
  * f2fs: move discard parameters into discard_cmd_control
  * f2fs: do not allow partial truncation on pinned file
  * icnss: Add code to pass device configs to wlan driver
  * fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs
  * msm: kgsl: Zap performance counters across context switches
  * msm: kgsl: Add a sysfs node to control performance counter reads
  * disp: uapi: drm: Add new blend type for CAC
  * Merge "msm: npu: Avoid buffer overflow when handling get_property packet"
  * Merge "USB: uac1: Fix audio quality issues for UAC1"
  * Merge "msm_bus: Check if cldata->pdata is null before access it"
  * version 6
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * USB: uac1: Fix audio quality issues for UAC1
  * defconfig: Enable UAC1 config for kona
  * UAC1: Add configfs attribute for changing ep max packet size
  * USB: f_uac1: Add check before accessing members of uac1
  * USB: f_uac1: add iad descriptor for UAC1 driver
  * f_uac1: Remove unwanted descriptors for volume/mute support
  * f_uac1: Add support for volume/mute control settings
  * msm: npu: Avoid buffer overflow when handling get_property packet
  * sctp: add param size validation for SCTP_PARAM_SET_PRIMARY
  * sctp: validate chunk size in __rcv_asconf_lookup
  * sctp: add size validation when walking chunks
  * sctp: validate from_addr_param return
  * aio: fix use-after-free due to missing POLLFREE handling
  * aio: keep poll requests on waitqueue until completed
  * signalfd: use wake_up_pollfree()
  * binder: use wake_up_pollfree()
  * wait: add wake_up_pollfree()
  * FROMGIT: USB: gadget: bRequestType is a bitfield, not a enum
  * UPSTREAM: USB: gadget: zero allocate endpoint 0 buffers
  * UPSTREAM: USB: gadget: detect too-big endpoint 0 requests
  * Merge "cnss2: Handle event processing after shutdown"
  * Merge "soc: qcom: socinfo: Add soc information for KhajeP and KhajeQ"
  * Merge "i2c: i2c-msm-v2: Set frequency parameters to INT and validate"
  * Merge d09499e9c8a99763bf9afa92c06601070523dc3e on remote branch
  * Merge "drivers: thermal: validate cdev sysfs state request before using it"
  * soc: qcom: socinfo: Add soc information for KhajeP and KhajeQ
  * msm_bus: Check if cldata->pdata is null before access it
  * spi: spi-msm-geni: Put device to suspend if PM status is active
  * Merge "cfg80211: Add support to advertize OCV support"
  * Merge "spi: spi-msm-geni: Protect from register access in suspend state"
  * Merge "spi: spi-msm-geni: Keep device to suspend if PM call fails"
  * cnss2: Handle event processing after shutdown
  * cfg80211: Add support to advertize OCV support
  * i2c: i2c-msm-v2: Set frequency parameters to INT and validate
  * msm: kgsl: Reduce latency while processing ioctls
  * setlocalversion: Never append a "+" to localversion
  * drivers:block:zram:default lz4
  * zram: Move default compression algorithm choice to Kconfig
  * defconfig: enable lz4 compression
  * drivers: thermal: validate cdev sysfs state request before using it
  * defconfigs: disable msm performance
  * USB: f_fs: Fix disconnect check during ongoing IO
  * Merge "msm: diag: fix copyright"
  * Merge "drivers: soc: qcom: Initialize blocking notifier as per lockdep"
  * msm: diag: fix copyright
  * Merge "diag: Validate the dci client before sending dci packet"
  * diag: Validate the dci client before sending dci packet
  * drivers: soc: qcom: Initialize blocking notifier as per lockdep
  * version 5
  * defconfig: regen
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * usb: gadget: u_audio: fix race condition on endpoint stop
  * usb: gadget: u_audio: Free requests only after callback
  * defconfig: Enable usb peripheral audio on SXR2130
  * Merge "diag: Ensure dci entry is valid before sending the packet"
  * Merge "msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()"
  * msm: ADSPRPC: Fix to avoid Use after free in fastrpc_init_process
  * Merge "msm: synx: fix copyright"
  * Merge "msm: ipa3: Fix to validate the NAT table entries during NAT table init"
  * msm: synx: fix copyright
  * clk: qcom: vdd-level: Update the vdd level for CX on Khaje
  * msm: ipa3: Fix to validate the NAT table entries during NAT table init
  * Merge 533e678218e41df69501158ff6eb84deafb6771c on remote branch
  * msm: synx: fix copyright
  * clk: qcom: gpucc: Update the vdd level for CX on Khaje
  * diag: Ensure dci entry is valid before sending the packet
  * Merge "usb: pd: Process request message as soon as it is received"
  * usb: pd: Process request message as soon as it is received
  * Merge "dfc: reset tx queue"
  * mmc: sdhci-msm: Update the MGPI SW check to avoid irq timeout
  * msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()
  * Merge "msm: ipa3: fix to cleanup the dma allocation"
  * Merge "msm: synx: remove synx handle details from logging"
  * dfc: reset tx queue
  * clk: qcom: gpucc: Add support for higher frequency for Khaje
  * Merge "clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje"
  * msm: ipa3: fix to cleanup the dma allocation
  * Merge "include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner"
  * clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje
  * Merge "msm: adsprpc: Handle UAF in fastrpc debugfs read"
  * include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner
  * Merge "usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind"
  * usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind
  * version 4
  * firmware: focaltech: Upgrade FW from haydin
  * touchscreen: focaltech_spi: Upgrade regardless of FW version check
  * techpack: audio: Mark msm-dai-q6 drivers as sync probe
  * techpack/audio: kona: Report correct key code of headset buttons
  * thermal: qcom: bcl: Return before thermal trip evaluates
  * defconfigs: enable Simple MSM Thermal
  * ARM64: dts: umi-sm8250: configure msm-thermal-simple
  * msm: thermal: simple: Introduce simple MSM thermal solution [msm-4.14]
  * version 3
  * drivers: thermal: limits-dcvs: Always build driver
  * drivers: power: supply: Disable battery capacity learning
  * defconfig: revert back to 100Hz tick
  * build: version 2
  * build: its Snow time !!!
  * drivers: usb: dwc3: Specify sync probe for dwc3-of-simple.
  * drivers: usb: dwc3: Specify sync probe for msm-dwc3 driver
  * drivers: usb: dwc3: Specify sync probe for dwc3 driver
  * drivers: usb: pd: remove fix pd output for 5v
  * Revert "usb: pd: Register typec partner in case AAA is connected"
  * PM / sleep: Skip OOM killer toggles when kernel is compiled for Android
  * drm: msm: optimize interpolation
  * techpack: drm: sde: fix a race condition
  * platform: msm: Fix dangerous relocation
  * defconfig: Bump SLMK minfree & timeout
  * power: process: Use lesser time to enter sleep
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * qos: Don't allow userspace to impose restrictions on CPU idle levels
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * dts: kona-gpu: Remove qos active latency node
  * treewide: use mi drm notifier
  * drivers: power: add timeouts to wakelocks
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * ARM64: umi: overlay gpu data from kona-v2-gpu.dtsi
  * arm64: dts: kona-v2-gpu: Overclock to 670mhz
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * gitignore: add out directory
  * add yarpiins build stuff
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/defconfig: apollo: Bringup custom defconfig
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * Revert "scsi: ufs: increase query timeout"
  * Revert "drivers: Add support for memory fragmentation test simulator driver"
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * ANDROID: sched: Exempt paused CPU from nohz idle balance
  * ARM64: configs: Make configs required to pass CTS tests mandatory
  * ARM64/defconfig: umi: Disable LSE atomics
  * ARM64/defconfig: umi: Use a timer frequency of 100 Hz
  * arm64: Inline the spin lock function family
  * ARM64/defconfig: umi: Disable unused errata
  * ARM64/defconfig: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * UPSTREAM: arm64: link with -z norelro for LLD or aarch64-elf
  * techpack: display: msm: dsi: set doze brightness for aod correctly
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64/defconfig: umi: Disable some unuse drivers
  * ARM64/defconfig: umi: Disable stability debug configs Disable the following stability debug configs for shipping: - CONFIG_EDAC_KRYO_ARM64_PANIC_ON_UE
  * ARM64/defconfig: umi: Enable LLD, RELR, Clang ThinLTO optimizations
  * ANDROID: sched: EAS: take cstate into account when selecting idle core
  * ARM64/defconfig: umi: Enable jump label
  * arm64/kernel: jump_label: Switch to relative references
  * locking/static_key: Add support for deferred static branches
  * jump_label: Fix NULL dereference bug in __jump_label_mod_update()
  * jump_label: Annotate entries that operate on __init code earlier
  * jump_label: Implement generic support for relative references
  * jump_label: Abstract jump_entry member accessors
  * jump_label: Use static_key_linked() accessor
  * mm/vmalloc.c: switch to WARN_ON() and move it under unlink_va()
  * mm/vmalloc.c: get rid of one single unlink_va() when merge
  * mm/vmalloc.c: preload a CPU with one object for split purpose
  * mm/vmalloc.c: remove "node" argument
  * mm/vmalloc.c: fix typo in comment
  * vmalloc: export __vmalloc_node_range for CONFIG_TEST_VMALLOC_MODULE
  * mm/vmalloc: pass VM_USERMAP flags directly to __vmalloc_node_range()
  * mm/vmalloc: do not call kmemleak_free() on not yet accounted memory
  * mm/vmalloc.c: make vmalloc_32_user() align base kernel virtual address to SHMLBA
  * mm: convert totalram_pages and totalhigh_pages variables to atomic
  * vfree: add debug might_sleep()
  * mm/vmalloc.c: improve vfree() kerneldoc
  * drivers: scsi: ufs: Remove unused d_lu_wb_buf_alloc
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64/defconfig: umi: Disable QHEE kernel memory protection
  * qcacld-3.0: Free a bunch of pkts at once
  * Makefile: Use O3 optimization level for Clang LTO
  * Makefile: Use -O3 optimization level
  * drivers: thermal: Don't qualify thermal polling as high priority
  * ARM64/defconfig: umi: Enable power efficient workqueues
  * ARM64/defconfig: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64/defconfig: Disable serial console support
  * arm64: Don't build legacy QCOM DTS.
  * rcu: fix a performance regression
  * qcacmn: Fix build error with !IPA_OFFLOAD
  * ARM64/defconfig: umi: Disable qti core control and sched autogroup
  * ARM: dts: kona: Disable IRQ debugging
  * ARM64/defconfig: umi: Disable some debug drivers
  * f2fs: Demote GC thread to idle scheduler class
  * f2fs: Set ioprio of GC kthread to idle
  * f2fs: Enlarge min_fsync_blocks to 20
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * UPSTREAM: zram: move backing_dev under macro CONFIG_ZRAM_WRITEBACK
  * UPSTREAM: zram: fix broken page writeback
  * UPSTREAM: zram: fix return value on writeback_store
  * UPSTREAM: zram: support page writeback
  * BACKPORT: zcomp: Use ARRAY_SIZE() for backends list
  * BACKPORT: zram: Allocate struct zcomp_strm as per-CPU memory
  * Revert "zram: introduce zram_entry to prepare dedup functionality"
  * Revert "zram: implement deduplication in zram"
  * Revert "zram: make deduplication feature optional"
  * Revert "zram: compare all the entries with same checksum for deduplication"
  * Revert "zram: fix race condition while returning zram_entry refcount"
  * ASoC: pcm: Add 24bit playback audio support
  * ARM64: configs: enable CONFIG_WIREGUARD
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * arm64/defconfig: umi: Enable crypto LZ4
  * arm64/defconfig: umi: Enable zram-writeback support
  * ARM64: umi/defconfigs: Bringup custom config
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
  * block: zram: Fix idle/writeback string compare
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * drivers: gpu: msm: Only build adreno 6xx part
  * video: backlight: disable modules enabled by default
  * media: gspca: disable gspca module
  * net: disable bridge netfilter module
  * net: ipv4: disable modules which are enabled by default
  * qcacmn: Fix suspicious string concatenation warning in fwlog
  * rmnet_shs: Fix CFI violation in packet assignment
  * rmnet_perf: Fix CFI violation in packet deaggregation
  * ntfs: Fix ntfs_test_inode and ntfs_init_locked_inode function type If the kernel is built with CFI we hit a __cfi_check_fail while mounting a partition
  * power: supply: Classify Battery Monitor Systems as batteries
  * cpufreq: stats: Replace the global lock with atomic.
  * techpack: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * techpack: camera: Unmap secure buffers in secure usecase
  * subsystem_restart: Always performs soft resets when subsystems crash
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * platform: msm: gsi: Export symbols only if compiled as module
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * scsi: ufs: Force enable write booster feature on UFS 3.1
  * scsi: ufs: Disable write booster feature support on UFS 2.2
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * treewide: Remove all Android.mk files
  * techpack: audio: Silence some logspam
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: Remove xiaomi early fingerprint wakeup optimization
  * audio: swr-mstr-ctrl: Fix unbalanced IRQ condition in swrm_runtime_suspend()
  * ARM64: configs: Enable XFRM_MIGRATE
  * ARM64: configs: Remove some debugging related features
  * ARM64: configs: Enable ARM64 Crypto Extensions SM3/4 and CRC32
  * ARM64: configs: Enable NTFS Filesystem
  * ARM64: configs: Enable ExFAT Filesystem
  * msm-poweroff: Store restart reason in panic
  * printk: Increase kernel log buffer size
  * smb5: Fix SDP current override for USB phy speed
  * mm: Disable watermark boost feature for K4.19
  * ARM64: configs: Skip file system sync in suspend
  * ARM64: configs: Disable PASR on kona devices
  * ARM64: configs: Reduce ION pool fill mark to 48MB
  * ARM64: configs: Build seperate DTB/DTBOs
  * ARM64: configs: Concatenate dtbs inside kernel image
  * ARM64: configs: Build Compressed Kernel Image
  * ARM64: configs: Build Qualcomm Atheros CLD WLAN module
  * ARM64: configs: Enable CPU frequency transition statistics
  * ARM64: configs: thyme: Enable support for block device writeback throttling
  * ARM64: configs: lmi: Enable AW2015 RGB LED support
  * ARM64: configs: lmi: Enable camera motor drivers
  * ARM64: configs: cas: Enable thermal dimming
  * ARM64: configs: apollo,cas: Enable panic on oops
  * ARM64: configs: apollo: Enable GPIO testing mode driver
  * ARM64: configs: Increase FRAME_WARN size to 4096
  * ARM64: configs: Enable Dynamic printk support
  * ARM64: configs: Disable MSM 11AD chip support
  * ARM64: configs: Enable xiaomi touchfeature driver
  * ARM64: configs: Disable USB CDC ADM support
  * ARM64: configs: Disable APP Armor security
  * ARM64: configs: Enable Ultrasound proximity driver
  * ARM64: configs: Enable USB Remote NDIS support
  * ARM64: configs: Disable USB Serial converter support
  * ARM64: configs: Disable V4L2 Videobuf2 Core
  * ARM64: configs: Enable SPI connected IR LED support
  * ARM64: configs: Enable PM8008 regulator driver
  * ARM64: configs: Enable device specific power supply drivers
  * ARM64: configs: Enable high performance logchar driver
  * ARM64: configs: Enable fastboot oem uart-enable command support
  * ARM64: configs: Disable support for Virtual terminal
  * ARM64: configs: Enable device specific fingerprint drivers
  * ARM64: configs: Enable AWINIC AW8697 haptics driver
  * ARM64: configs: Disable QTI Haptics support
  * ARM64: configs: Enable device specific touchscreen drivers
  * ARM64: configs: Disable Open-channel SSD target support
  * ARM64: configs: Enable CNSS QCA6390 chipset support
  * ARM64: configs: Enable QTI Smart Link Aggregation driver
  * ARM64: configs: Enable NAT, PEDIT and CSUM classifier Actions
  * ARM64: configs: Disable DEBUG_FS
  * ARM64: configs: Build Wilocity wil6210 driver
  * ARM64: configs: Enable Memory Technology Device (MTD) support
  * ARM64: configs: Disable Modules signature
  * ARM64: configs: Enable pstore support
  * ARM64: configs: Increase Kernel log buffer size
  * ARM64: configs: Enable Audit support
  * ARM64: configs: Enable device specific board platform
  * ARM64: configs: Generate xiaomi sm8250 devices configs from kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * bpf: Get the comm hash of the socket process stored inside sk buffer
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
  * fs: fuse: Implement FUSE passthrough
  * disp: msm: sde: Force SDE fd to start from 1
  * firmware: Import xiaomi touchscreens firmware
  * video: backlight: qcom-spmi-wled: Disable CABC on low brightness
  * video: backlight: Add support for thermal backlight dimming on J1S
  * video: backlight: Create brightness clone sysfs node
  * video: backlight: Notify on brightness node change
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * staging: android: ion: adjust system heap pool orders
  * staging: android: ion: Add xiaomi ion camera heap modifications
  * spi: gen-qcom: increase transfer timeout
  * soc: qcom: Add socinfo support for xiaomi sm8250 and sm7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
  * scsi: ufs: increase query timeout
  * mtd: devices: block2mtd: Sync scsi I/O on panic
  * debug: Add support to dump kmsg logs in kernel panic/oops using mtdoops
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * pinctrl: msm: Disable access to specified reserved gpios
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * cnss2: Add support for loading different bdwlan firmwares
  * net: cnss2: Add paramter to disable NV MAC support
  * net: cnss2: Increase cnss-daemon qmi timeout
  * media: rc: Add xiaomi modifications to IR driver
  * media: msm: cvp: Replace fence request handler kthread with workqueue
  * drivers: leds: Allow switching between dual flashlight leds
  * drivers: leds: Add support for AWINIC AW2015 3 Channel LED driver
  * drivers: cpuidle: Add xiaomi parameter to disable cpuidle and lpm
  * drivers: iio: Add support for Xiaomi ultrasound proximity sensor
  * drm: msm: Import xiaomi drm modifications
  * thermal: bcl_pmic5: Apply xiaomi thermal modifications
  * cpufreq: qcom: Always report maximum value of freq in limits_mitigation_notify()
  * drivers: thermal: Switch to CPUFREQ_ADJUST
  * mhi: mhi_qcom: Apply xiaomi modifications to sdx55 modem
  * drivers: misc: Import xiaomi GPIO testing mode driver
  * drivers: misc: Import Texas Instruments H-Bridge Stepper Motor Driver
  * drivers: misc: Import AKM Sensortecs AK09970 HALL sensor driver
  * block: Disable preemption before request_fn calls
  * ARM64: dts: qcom: Import kona mtp devicetree
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * input: touchscreen: Add support for xiaomi touchfeature dirver
  * input: touchscreen: Add support for Novatek NT36672c SPI touchscreen driver
  * input: touchscreen: Add support for GOODIX GT9886 touchscreen driver
  * input: touchscreen: Add support for STM FTS521 touchscreen driver
  * input: touchscreen: Add support for FocalTech ft8719/ft5452 touchscreen drivers
  * input: touchscreen: focaltech_spi: Don't ignore firmware files
  * input: touchscreen: Add support for FocalTech ft3658 touchscreen driver
  * input: touchscreen: Disable caf default touchscreen drivers
  * input: misc: aw8697_haptic: Add support to reload firmware for umi
  * input: misc: Add support for AWINIC AW8696 Haptic driver
  * input: fingerprint: Add support for GOODIX FOD fingerprint driver
  * input: fingerprint: Add support for GOODIX fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * drivers: Add support for memory fragmentation test simulator driver
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * techpack: audio: nuke xlogchar
  * Merge commit '99646a7c5d9584934073bd5c49438993f44c2f31' into android11-base
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * Add 'arch/arm64/boot/dts/vendor/' from commit 'bd2d6b0afa1f8aba19e41ce3bc29c16745595efa'
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * techpack: data: Build high performance ipa/rmnet drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * Revert "selinux: Relocate ss_initialized and selinux_enforcing to separate 4k"
  * ipa3: fix improper size checks
  * msm: ipa: Fix Makefile
  * kbuild: Remove gcc-wrapper
  * add toggle for disabling newly added USB devices
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * techpack/pinctrl-lpi: initialise at late_initcall
  * techpack: audio: makefile: do not export all the variables
  * techpack: audio: Correct symlinks
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into lineage-19.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/camera-kernel into 'LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-00300-XXXX.QSSI12.0
  * msm: adsprpc: Handle UAF in fastrpc debugfs read
  * usb: gadget: f_hid: Fix unbind issues
  * usb: gadget: f_uvc: Fix unbind issues
  * Merge "clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI"
  * msm: synx: remove synx handle details from logging
  * clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI
  * Merge "input: misc: qcom-power-on: Add support to log KPDPWR status"
  * Merge "msm: kgsl: Fix gpuaddr_in_range() to check upper bound"
  * mmc: sdhci-msm: Ensure SD card power isn't ON when card removed
  * Merge "clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL"
  * msm: kgsl: Fix gpuaddr_in_range() to check upper bound
  * Merge "usb: max-3421: Prevent corruption of freed memory"
  * clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL
  * spi: spi-msm-geni: Protect from register access in suspend state
  * spi: spi-msm-geni: Keep device to suspend if PM call fails
  * Merge "regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling"
  * Merge "seq_file: disallow extremely large seq buffer allocations"
  * regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling
  * Merge a425c3837db7a914ad21e6f5634b4c6dc7083b43 on remote branch
  * usb: gadget: f_uac1: Add support for UAC1 function
  * Merge "tzlog: update struct to get normal and fatal diag logs"
  * Merge "fs: crypto: Maintain reference count for class keys"
  * input: misc: qcom-power-on: Add support to log KPDPWR status
  * Merge "diag: Update log and event mask code ranges"
  * tzlog: update struct to get normal and fatal diag logs
  * firmware: qcom: Remove garbage characters from qsee log
  * firmware: qcom: add enlarged qsee log support
  * firmware: qcom: encrypted tz and qsee log support
  * fs: crypto: Maintain reference count for class keys
  * Merge "usb: pd: always log when state machine work is queued"
  * defconfig: arm: msm: Added spmi_sdam support for SDM429
  * usb: pd: always log when state machine work is queued
  * leds: qti-tri-led: remove LED_KEEP_TRIGGER flag
  * usb: pd: add usbpd_dbg() in pd_send_msg()
  * Merge "mhi: core: Increase RDDM timeout to 350ms"
  * diag: Update log and event mask code ranges
  * usb: gadget: Clear string index for RMNET
  * Merge "drivers: usb: enable SET_REPORT through EP0 feature"
  * drivers: usb: enable SET_REPORT through EP0 feature
  * Merge "af_unix: fix garbage collect vs MSG_PEEK"
  * Merge "mmc: host: Add recovery_finish crypto vops"
  * Merge "dfc: hold wakelock while powersave timer is running"
  * Merge "uapi: sound: add bt external sink delay parameter"
  * Merge "msm: npu: remove asynchronous network execution support"
  * Merge "diag: Avoid possible out of bound access while sending masks"
  * Merge "diag: Sanitize non-hdlc pkt length against buffer capacity"
  * Merge "diag: Allow DCI packets to be copied separately"
  * Merge "clk: qcom: debugcc: Remove im_sleep/xo_div4 for QM215"
  * Merge "msm: gsi: Stop Channel support when in Flow Control State"
  * clk: qcom: debugcc: Remove gcc_pcnoc_mpu_cfg_ahb_clk for QM215
  * clk: qcom: debugcc: Remove im_sleep/xo_div4 for QM215
  * af_unix: fix garbage collect vs MSG_PEEK
  * net: split out functions related to registering inflight socket files
  * uapi: sound: add bt external sink delay parameter
  * mhi: core: Increase RDDM timeout to 350ms
  * msm: npu: remove asynchronous network execution support
  * usb: max-3421: Prevent corruption of freed memory
  * mmc: host: Add recovery_finish crypto vops
  * seq_file: disallow extremely large seq buffer allocations
  * dfc: hold wakelock while powersave timer is running
  * diag: Avoid possible out of bound access while sending masks
  * diag: Sanitize non-hdlc pkt length against buffer capacity
  * diag: Allow DCI packets to be copied separately
  * msm: gsi: Stop Channel support when in Flow Control State
  * msm: kgsl: Signal fence only if last fence refcount was not put
  * pci_iomap: fix page fault issue on vmalloc with section mapping
  * Merge "msm: adsprpc: Handle UAF in process shell memory"
  * media: v4l2: Allow ioctl type of "U" for video devices like usb camera
  * msm: adsprpc: Handle UAF in process shell memory
  * Merge commit '82146398a44b8081ef002068af683b1f44f4b226' into kernel.lnx.4.19.r23-rel
  * Merge "cnss2: Check if firmware asserts before power off for CBC"
  * Merge "net: qrtr: Use radix_tree_iter_delete to delete tx flow"
  * Merge "net: qrtr: Cleanup flow control during DEL proc"
  * cnss2: Check if firmware asserts before power off for CBC
  * Merge "net: qrtr: Cleanup flow control during remote socket release"
  * Merge "usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path"
  * usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path
  * net: qrtr: Use radix_tree_iter_delete to delete tx flow
  * net: qrtr: Cleanup flow control during DEL proc
  * net: qrtr: Cleanup flow control during remote socket release
  * net: qrtr: Converting DEL_PROC command to BYE command
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * defconfig: Enable Novatek NT36xxx Touch for tron target
  * cnss2: Update bound checks for sbl reg dumps to SRAM mem range

n.n.n / 2022-04-25
==================

  * ARM64: kona-v2.1: fix max frequency on thermals
  * ARM64: dts: xiaomi-sm8250-common: configure msm-thermal-simple
  * Revert "ARM64: dts: umi-sm8250: configure msm-thermal-simple"
  * arm64: dts: xiaomi: Allow big cluster to idle in USB perf mode
  * defconfigs: enable pelt halflife
  * ANDROID: sched/fair: correct pelt load information in sched-pelt.h
  * FROMLIST: sched/fair: add support to tune PELT ramp/decay timings
  * sched/fair: Fix kernel warning
  * UPSTREAM: sched: Fix out-of-bound access in uclamp
  * BACKPORT: sched/fair: Fix overutilized update in enqueue_task_fair()
  * kernel/sched: spread big prefer_idle tasks to little cores
  * UPSTREAM: sched/rt: Disable RT_RUNTIME_SHARE by default
  * sched: refine code for computing energy
  * sched/fair: refine can_migrate_boosted_task
  * sched/core: fix userspace affining threads incorrectly by task name.
  * kernel: sched: account for real time utilization
  * sched: Improve the scheduler
  * sched/core: Fix use after free issue in is_sched_lib_based_app()
  * sched/core: fix userspace affining threads incorrectly
  * sched: fine tune task placement for prioritized tasks
  * sched: fair: placement optimization for heavy load
  * Revert "sched: fine tune task placement for prioritized tasks"
  * sched: fine tune task placement for prioritized tasks
  * sched/fair: schedule lower priority tasks from little cores
  * sched/fair: do not use boosted margin for prefer_high_cap case
  * sched/fair: use actual cpu capacity to calculate boosted util
  * sched: separate capacity margin for boosted tasks
  * sched: separate boost signal from placement hint
  * kernel: sched: merge changes from LA.UM.9.12.R2.10.00.00.685.011
  * Use find_best_target to select cpu for a zero-util task
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * GKI: sched: Add back the root_domain.overutilized field
  * GKI: sched: Compile out push_task field in struct rq
  * sched: restrict iowait boost to tasks with prefer_idle
  * trace: sched: add capacity change tracing
  * sched: reduce softirq conflicts with RT
  * sched/fair: let scheduler skip util checking if cpu is idle
  * kernel: sched: Mitigate non-boosted tasks preempting boosted tasks
  * Revert "sched/core: fix userspace affining threads incorrectly"
  * Revert "sched/core: Fix use after free issue in is_sched_lib_based_app()"
  * Revert "sched: Improve the scheduler"
  * sched/fair: prefer exclusive mid cluster cpu for top-app task
  * sched: delete unused & buggy function definitions
  * sched/fair: fix implementation of is_min_capacity_cpu()
  * sched/fair: refine some scheduler changes from AU drop
  * BACKPORT: sched/fair: if sync flag ignored, try to place in mid cluster
  * Revert "sched/walt: Improve the scheduler"
  * sched: core: Disable double lock/unlock balance in move_queued_task()
  * sched: fair: Disable double lock/unlock balance in detach_task()
  * sched/fair: apply sync wake-up to pure CFS path
  * Revert "sched: fair: Always try to use energy efficient cpu for wakeups"
  * sched: fair: avoid little cpus due to sync, prev bias
  * sched: fix issue of cpu freq running at max always
  * sched/fair: Fix compilation issues for !CONFIG_SCHED_WALT
  * kernel: sched: fix cpu cpu_capacity_orig being capped incorrectly
  * sched/walt: Fix negative count of sched_asym_cpucapacity static key
  * Revert "sched/fair: Add policy for restricting prefer_spread to newly idle balance"
  * fs: ext4: fsync: optimize double-fsync() a bunch
  * block : makefile : disable align mismatch
  * blk-mq: fix corruption with direct issue
  * ashmem: Adapt building on msm-4.19
  * ashmem: Rewrite to improve clarity and performance
  * block: cfq-iosched: Port samsung optimizations from SM-N986B
  * cpufreq: schedutil: Smoothen WALT predicted load boosting
  * cpufreq: schedutil: fix check for stale utilization values
  * sched: Do not reduce perceived CPU capacity while idle
  * sched/features: Fix hrtick reprogramming
  * sched: features: Disable EAS_PREFER_IDLE
  * arch: arm64: dts: qcom: kona-camera: fix i2c and order clocks correctly
  * techpack: display: Store dc status when fod hbm enabled
  * techpack: display: dsi_phy: Force allow panel phy power off
  * techpack: display: dsi_panel: Force enable Ultra-Low Power State feature for all panel
  * version 7
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * sysfs: Add sysfs_emit and sysfs_emit_at to format sysfs output
  * Revert "Add toggle for disabling newly added USB devices"
  * cpufreqX: schedutil: Use kobject release() method to free sugov_tunables
  * f2fs: avoid infinite loop to flush node pages
  * f2fs: fix wrong condition check when failing metapage read
  * f2fs: remove unnecessary f2fs_lock_op in f2fs_new_inode
  * f2fs: fix to do sanity check on .cp_pack_total_block_count
  * f2fs: compress: fix to print raw data size in error path of lz4 decompression
  * f2fs: remove redundant parameter judgment
  * drivers: input: aw8697: Fix missmerge
  * ARM64: dts: Use new awinic haptics configuration
  * input: misc: aw8697_haptic: Upgrade from zeus-s-oss
  * arch: dts: Throttle charging speed and Optimise charging current
  * msm: thermal: simple: account for battery temperature in calculations
  * arm64: lib: Memory utilities optimization
  * proc: do not let google camera run in the background
  * thermal: Increase thermal trip points to 16
  * f2fs: use spin_lock to avoid hang
  * f2fs: remove unnecessary read for F2FS_FITS_IN_INODE
  * f2fs: avoid an infinite loop in f2fs_sync_dirty_inodes
  * f2fs: fix to do sanity check on curseg->alloc_type
  * f2fs: fix to avoid potential deadlock
  * f2fs: fix missing free nid in f2fs_handle_failed_inode
  * f2fs: add a way to limit roll forward recovery time
  * f2fs: adjust readahead block number during recovery
  * f2fs: expose discard related parameters in sysfs
  * f2fs: move discard parameters into discard_cmd_control
  * f2fs: do not allow partial truncation on pinned file
  * version 6
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge d09499e9c8a99763bf9afa92c06601070523dc3e on remote branch
  * spi: spi-msm-geni: Put device to suspend if PM status is active
  * Merge "cfg80211: Add support to advertize OCV support"
  * Merge "spi: spi-msm-geni: Protect from register access in suspend state"
  * Merge "spi: spi-msm-geni: Keep device to suspend if PM call fails"
  * cfg80211: Add support to advertize OCV support
  * msm: kgsl: Reduce latency while processing ioctls
  * setlocalversion: Never append a "+" to localversion
  * drivers:block:zram:default lz4
  * zram: Move default compression algorithm choice to Kconfig
  * defconfig: enable lz4 compression
  * defconfigs: disable msm performance
  * USB: f_fs: Fix disconnect check during ongoing IO
  * Merge "msm: diag: fix copyright"
  * Merge "drivers: soc: qcom: Initialize blocking notifier as per lockdep"
  * msm: diag: fix copyright
  * Merge "diag: Validate the dci client before sending dci packet"
  * diag: Validate the dci client before sending dci packet
  * drivers: soc: qcom: Initialize blocking notifier as per lockdep
  * version 5
  * defconfig: regen
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * usb: gadget: u_audio: fix race condition on endpoint stop
  * usb: gadget: u_audio: Free requests only after callback
  * defconfig: Enable usb peripheral audio on SXR2130
  * Merge "diag: Ensure dci entry is valid before sending the packet"
  * Merge "msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()"
  * msm: ADSPRPC: Fix to avoid Use after free in fastrpc_init_process
  * Merge "msm: synx: fix copyright"
  * Merge "msm: ipa3: Fix to validate the NAT table entries during NAT table init"
  * msm: synx: fix copyright
  * clk: qcom: vdd-level: Update the vdd level for CX on Khaje
  * msm: ipa3: Fix to validate the NAT table entries during NAT table init
  * Merge 533e678218e41df69501158ff6eb84deafb6771c on remote branch
  * msm: synx: fix copyright
  * clk: qcom: gpucc: Update the vdd level for CX on Khaje
  * diag: Ensure dci entry is valid before sending the packet
  * Merge "usb: pd: Process request message as soon as it is received"
  * usb: pd: Process request message as soon as it is received
  * Merge "dfc: reset tx queue"
  * mmc: sdhci-msm: Update the MGPI SW check to avoid irq timeout
  * msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()
  * Merge "msm: ipa3: fix to cleanup the dma allocation"
  * Merge "msm: synx: remove synx handle details from logging"
  * dfc: reset tx queue
  * clk: qcom: gpucc: Add support for higher frequency for Khaje
  * Merge "clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje"
  * msm: ipa3: fix to cleanup the dma allocation
  * Merge "include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner"
  * clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje
  * Merge "msm: adsprpc: Handle UAF in fastrpc debugfs read"
  * include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner
  * Merge "usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind"
  * usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind
  * version 4
  * firmware: focaltech: Upgrade FW from haydin
  * touchscreen: focaltech_spi: Upgrade regardless of FW version check
  * techpack: audio: Mark msm-dai-q6 drivers as sync probe
  * techpack/audio: kona: Report correct key code of headset buttons
  * thermal: qcom: bcl: Return before thermal trip evaluates
  * defconfigs: enable Simple MSM Thermal
  * ARM64: dts: umi-sm8250: configure msm-thermal-simple
  * msm: thermal: simple: Introduce simple MSM thermal solution [msm-4.14]
  * version 3
  * drivers: thermal: limits-dcvs: Always build driver
  * drivers: power: supply: Disable battery capacity learning
  * defconfig: revert back to 100Hz tick
  * build: version 2
  * build: its Snow time !!!
  * drivers: usb: dwc3: Specify sync probe for dwc3-of-simple.
  * drivers: usb: dwc3: Specify sync probe for msm-dwc3 driver
  * drivers: usb: dwc3: Specify sync probe for dwc3 driver
  * drivers: usb: pd: remove fix pd output for 5v
  * Revert "usb: pd: Register typec partner in case AAA is connected"
  * PM / sleep: Skip OOM killer toggles when kernel is compiled for Android
  * drm: msm: optimize interpolation
  * techpack: drm: sde: fix a race condition
  * platform: msm: Fix dangerous relocation
  * defconfig: Bump SLMK minfree & timeout
  * power: process: Use lesser time to enter sleep
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * qos: Don't allow userspace to impose restrictions on CPU idle levels
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * dts: kona-gpu: Remove qos active latency node
  * treewide: use mi drm notifier
  * drivers: power: add timeouts to wakelocks
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * ARM64: umi: overlay gpu data from kona-v2-gpu.dtsi
  * arm64: dts: kona-v2-gpu: Overclock to 670mhz
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * gitignore: add out directory
  * add yarpiins build stuff
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/defconfig: apollo: Bringup custom defconfig
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * Revert "scsi: ufs: increase query timeout"
  * Revert "drivers: Add support for memory fragmentation test simulator driver"
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * ANDROID: sched: Exempt paused CPU from nohz idle balance
  * ARM64: configs: Make configs required to pass CTS tests mandatory
  * ARM64/defconfig: umi: Disable LSE atomics
  * ARM64/defconfig: umi: Use a timer frequency of 100 Hz
  * arm64: Inline the spin lock function family
  * ARM64/defconfig: umi: Disable unused errata
  * ARM64/defconfig: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * UPSTREAM: arm64: link with -z norelro for LLD or aarch64-elf
  * techpack: display: msm: dsi: set doze brightness for aod correctly
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64/defconfig: umi: Disable some unuse drivers
  * ARM64/defconfig: umi: Disable stability debug configs Disable the following stability debug configs for shipping: - CONFIG_EDAC_KRYO_ARM64_PANIC_ON_UE
  * ARM64/defconfig: umi: Enable LLD, RELR, Clang ThinLTO optimizations
  * ANDROID: sched: EAS: take cstate into account when selecting idle core
  * ARM64/defconfig: umi: Enable jump label
  * arm64/kernel: jump_label: Switch to relative references
  * locking/static_key: Add support for deferred static branches
  * jump_label: Fix NULL dereference bug in __jump_label_mod_update()
  * jump_label: Annotate entries that operate on __init code earlier
  * jump_label: Implement generic support for relative references
  * jump_label: Abstract jump_entry member accessors
  * jump_label: Use static_key_linked() accessor
  * mm/vmalloc.c: switch to WARN_ON() and move it under unlink_va()
  * mm/vmalloc.c: get rid of one single unlink_va() when merge
  * mm/vmalloc.c: preload a CPU with one object for split purpose
  * mm/vmalloc.c: remove "node" argument
  * mm/vmalloc.c: fix typo in comment
  * vmalloc: export __vmalloc_node_range for CONFIG_TEST_VMALLOC_MODULE
  * mm/vmalloc: pass VM_USERMAP flags directly to __vmalloc_node_range()
  * mm/vmalloc: do not call kmemleak_free() on not yet accounted memory
  * mm/vmalloc.c: make vmalloc_32_user() align base kernel virtual address to SHMLBA
  * mm: convert totalram_pages and totalhigh_pages variables to atomic
  * vfree: add debug might_sleep()
  * mm/vmalloc.c: improve vfree() kerneldoc
  * drivers: scsi: ufs: Remove unused d_lu_wb_buf_alloc
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64/defconfig: umi: Disable QHEE kernel memory protection
  * qcacld-3.0: Free a bunch of pkts at once
  * Makefile: Use O3 optimization level for Clang LTO
  * Makefile: Use -O3 optimization level
  * drivers: thermal: Don't qualify thermal polling as high priority
  * ARM64/defconfig: umi: Enable power efficient workqueues
  * ARM64/defconfig: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64/defconfig: Disable serial console support
  * arm64: Don't build legacy QCOM DTS.
  * rcu: fix a performance regression
  * qcacmn: Fix build error with !IPA_OFFLOAD
  * ARM64/defconfig: umi: Disable qti core control and sched autogroup
  * ARM: dts: kona: Disable IRQ debugging
  * ARM64/defconfig: umi: Disable some debug drivers
  * f2fs: Demote GC thread to idle scheduler class
  * f2fs: Set ioprio of GC kthread to idle
  * f2fs: Enlarge min_fsync_blocks to 20
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * UPSTREAM: zram: move backing_dev under macro CONFIG_ZRAM_WRITEBACK
  * UPSTREAM: zram: fix broken page writeback
  * UPSTREAM: zram: fix return value on writeback_store
  * UPSTREAM: zram: support page writeback
  * BACKPORT: zcomp: Use ARRAY_SIZE() for backends list
  * BACKPORT: zram: Allocate struct zcomp_strm as per-CPU memory
  * Revert "zram: introduce zram_entry to prepare dedup functionality"
  * Revert "zram: implement deduplication in zram"
  * Revert "zram: make deduplication feature optional"
  * Revert "zram: compare all the entries with same checksum for deduplication"
  * Revert "zram: fix race condition while returning zram_entry refcount"
  * ASoC: pcm: Add 24bit playback audio support
  * ARM64: configs: enable CONFIG_WIREGUARD
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * arm64/defconfig: umi: Enable crypto LZ4
  * arm64/defconfig: umi: Enable zram-writeback support
  * ARM64: umi/defconfigs: Bringup custom config
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
  * block: zram: Fix idle/writeback string compare
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * drivers: gpu: msm: Only build adreno 6xx part
  * video: backlight: disable modules enabled by default
  * media: gspca: disable gspca module
  * net: disable bridge netfilter module
  * net: ipv4: disable modules which are enabled by default
  * qcacmn: Fix suspicious string concatenation warning in fwlog
  * rmnet_shs: Fix CFI violation in packet assignment
  * rmnet_perf: Fix CFI violation in packet deaggregation
  * ntfs: Fix ntfs_test_inode and ntfs_init_locked_inode function type If the kernel is built with CFI we hit a __cfi_check_fail while mounting a partition
  * power: supply: Classify Battery Monitor Systems as batteries
  * cpufreq: stats: Replace the global lock with atomic.
  * techpack: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * techpack: camera: Unmap secure buffers in secure usecase
  * subsystem_restart: Always performs soft resets when subsystems crash
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * platform: msm: gsi: Export symbols only if compiled as module
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * scsi: ufs: Force enable write booster feature on UFS 3.1
  * scsi: ufs: Disable write booster feature support on UFS 2.2
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * treewide: Remove all Android.mk files
  * techpack: audio: Silence some logspam
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: Remove xiaomi early fingerprint wakeup optimization
  * audio: swr-mstr-ctrl: Fix unbalanced IRQ condition in swrm_runtime_suspend()
  * ARM64: configs: Enable XFRM_MIGRATE
  * ARM64: configs: Remove some debugging related features
  * ARM64: configs: Enable ARM64 Crypto Extensions SM3/4 and CRC32
  * ARM64: configs: Enable NTFS Filesystem
  * ARM64: configs: Enable ExFAT Filesystem
  * msm-poweroff: Store restart reason in panic
  * printk: Increase kernel log buffer size
  * smb5: Fix SDP current override for USB phy speed
  * mm: Disable watermark boost feature for K4.19
  * ARM64: configs: Skip file system sync in suspend
  * ARM64: configs: Disable PASR on kona devices
  * ARM64: configs: Reduce ION pool fill mark to 48MB
  * ARM64: configs: Build seperate DTB/DTBOs
  * ARM64: configs: Concatenate dtbs inside kernel image
  * ARM64: configs: Build Compressed Kernel Image
  * ARM64: configs: Build Qualcomm Atheros CLD WLAN module
  * ARM64: configs: Enable CPU frequency transition statistics
  * ARM64: configs: thyme: Enable support for block device writeback throttling
  * ARM64: configs: lmi: Enable AW2015 RGB LED support
  * ARM64: configs: lmi: Enable camera motor drivers
  * ARM64: configs: cas: Enable thermal dimming
  * ARM64: configs: apollo,cas: Enable panic on oops
  * ARM64: configs: apollo: Enable GPIO testing mode driver
  * ARM64: configs: Increase FRAME_WARN size to 4096
  * ARM64: configs: Enable Dynamic printk support
  * ARM64: configs: Disable MSM 11AD chip support
  * ARM64: configs: Enable xiaomi touchfeature driver
  * ARM64: configs: Disable USB CDC ADM support
  * ARM64: configs: Disable APP Armor security
  * ARM64: configs: Enable Ultrasound proximity driver
  * ARM64: configs: Enable USB Remote NDIS support
  * ARM64: configs: Disable USB Serial converter support
  * ARM64: configs: Disable V4L2 Videobuf2 Core
  * ARM64: configs: Enable SPI connected IR LED support
  * ARM64: configs: Enable PM8008 regulator driver
  * ARM64: configs: Enable device specific power supply drivers
  * ARM64: configs: Enable high performance logchar driver
  * ARM64: configs: Enable fastboot oem uart-enable command support
  * ARM64: configs: Disable support for Virtual terminal
  * ARM64: configs: Enable device specific fingerprint drivers
  * ARM64: configs: Enable AWINIC AW8697 haptics driver
  * ARM64: configs: Disable QTI Haptics support
  * ARM64: configs: Enable device specific touchscreen drivers
  * ARM64: configs: Disable Open-channel SSD target support
  * ARM64: configs: Enable CNSS QCA6390 chipset support
  * ARM64: configs: Enable QTI Smart Link Aggregation driver
  * ARM64: configs: Enable NAT, PEDIT and CSUM classifier Actions
  * ARM64: configs: Disable DEBUG_FS
  * ARM64: configs: Build Wilocity wil6210 driver
  * ARM64: configs: Enable Memory Technology Device (MTD) support
  * ARM64: configs: Disable Modules signature
  * ARM64: configs: Enable pstore support
  * ARM64: configs: Increase Kernel log buffer size
  * ARM64: configs: Enable Audit support
  * ARM64: configs: Enable device specific board platform
  * ARM64: configs: Generate xiaomi sm8250 devices configs from kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * bpf: Get the comm hash of the socket process stored inside sk buffer
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
  * fs: fuse: Implement FUSE passthrough
  * disp: msm: sde: Force SDE fd to start from 1
  * firmware: Import xiaomi touchscreens firmware
  * video: backlight: qcom-spmi-wled: Disable CABC on low brightness
  * video: backlight: Add support for thermal backlight dimming on J1S
  * video: backlight: Create brightness clone sysfs node
  * video: backlight: Notify on brightness node change
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * staging: android: ion: adjust system heap pool orders
  * staging: android: ion: Add xiaomi ion camera heap modifications
  * spi: gen-qcom: increase transfer timeout
  * soc: qcom: Add socinfo support for xiaomi sm8250 and sm7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
  * scsi: ufs: increase query timeout
  * mtd: devices: block2mtd: Sync scsi I/O on panic
  * debug: Add support to dump kmsg logs in kernel panic/oops using mtdoops
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * pinctrl: msm: Disable access to specified reserved gpios
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * cnss2: Add support for loading different bdwlan firmwares
  * net: cnss2: Add paramter to disable NV MAC support
  * net: cnss2: Increase cnss-daemon qmi timeout
  * media: rc: Add xiaomi modifications to IR driver
  * media: msm: cvp: Replace fence request handler kthread with workqueue
  * drivers: leds: Allow switching between dual flashlight leds
  * drivers: leds: Add support for AWINIC AW2015 3 Channel LED driver
  * drivers: cpuidle: Add xiaomi parameter to disable cpuidle and lpm
  * drivers: iio: Add support for Xiaomi ultrasound proximity sensor
  * drm: msm: Import xiaomi drm modifications
  * thermal: bcl_pmic5: Apply xiaomi thermal modifications
  * cpufreq: qcom: Always report maximum value of freq in limits_mitigation_notify()
  * drivers: thermal: Switch to CPUFREQ_ADJUST
  * mhi: mhi_qcom: Apply xiaomi modifications to sdx55 modem
  * drivers: misc: Import xiaomi GPIO testing mode driver
  * drivers: misc: Import Texas Instruments H-Bridge Stepper Motor Driver
  * drivers: misc: Import AKM Sensortecs AK09970 HALL sensor driver
  * block: Disable preemption before request_fn calls
  * ARM64: dts: qcom: Import kona mtp devicetree
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * input: touchscreen: Add support for xiaomi touchfeature dirver
  * input: touchscreen: Add support for Novatek NT36672c SPI touchscreen driver
  * input: touchscreen: Add support for GOODIX GT9886 touchscreen driver
  * input: touchscreen: Add support for STM FTS521 touchscreen driver
  * input: touchscreen: Add support for FocalTech ft8719/ft5452 touchscreen drivers
  * input: touchscreen: focaltech_spi: Don't ignore firmware files
  * input: touchscreen: Add support for FocalTech ft3658 touchscreen driver
  * input: touchscreen: Disable caf default touchscreen drivers
  * input: misc: aw8697_haptic: Add support to reload firmware for umi
  * input: misc: Add support for AWINIC AW8696 Haptic driver
  * input: fingerprint: Add support for GOODIX FOD fingerprint driver
  * input: fingerprint: Add support for GOODIX fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * drivers: Add support for memory fragmentation test simulator driver
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * techpack: audio: nuke xlogchar
  * Merge commit '99646a7c5d9584934073bd5c49438993f44c2f31' into android11-base
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * Add 'arch/arm64/boot/dts/vendor/' from commit 'bd2d6b0afa1f8aba19e41ce3bc29c16745595efa'
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * techpack: data: Build high performance ipa/rmnet drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * Revert "selinux: Relocate ss_initialized and selinux_enforcing to separate 4k"
  * ipa3: fix improper size checks
  * msm: ipa: Fix Makefile
  * kbuild: Remove gcc-wrapper
  * add toggle for disabling newly added USB devices
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * techpack/pinctrl-lpi: initialise at late_initcall
  * techpack: audio: makefile: do not export all the variables
  * techpack: audio: Correct symlinks
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into lineage-19.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/camera-kernel into 'LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-00300-XXXX.QSSI12.0
  * msm: adsprpc: Handle UAF in fastrpc debugfs read
  * usb: gadget: f_hid: Fix unbind issues
  * usb: gadget: f_uvc: Fix unbind issues
  * Merge "clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI"
  * msm: synx: remove synx handle details from logging
  * clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI
  * Merge "input: misc: qcom-power-on: Add support to log KPDPWR status"
  * Merge "msm: kgsl: Fix gpuaddr_in_range() to check upper bound"
  * mmc: sdhci-msm: Ensure SD card power isn't ON when card removed
  * Merge "clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL"
  * msm: kgsl: Fix gpuaddr_in_range() to check upper bound
  * Merge "usb: max-3421: Prevent corruption of freed memory"
  * clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL
  * spi: spi-msm-geni: Protect from register access in suspend state
  * spi: spi-msm-geni: Keep device to suspend if PM call fails
  * Merge "regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling"
  * Merge "seq_file: disallow extremely large seq buffer allocations"
  * regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling
  * Merge a425c3837db7a914ad21e6f5634b4c6dc7083b43 on remote branch
  * usb: gadget: f_uac1: Add support for UAC1 function
  * Merge "tzlog: update struct to get normal and fatal diag logs"
  * Merge "fs: crypto: Maintain reference count for class keys"
  * input: misc: qcom-power-on: Add support to log KPDPWR status
  * Merge "diag: Update log and event mask code ranges"
  * tzlog: update struct to get normal and fatal diag logs
  * firmware: qcom: Remove garbage characters from qsee log
  * firmware: qcom: add enlarged qsee log support
  * firmware: qcom: encrypted tz and qsee log support
  * fs: crypto: Maintain reference count for class keys
  * Merge "usb: pd: always log when state machine work is queued"
  * usb: pd: always log when state machine work is queued
  * leds: qti-tri-led: remove LED_KEEP_TRIGGER flag
  * usb: pd: add usbpd_dbg() in pd_send_msg()
  * Merge "mhi: core: Increase RDDM timeout to 350ms"
  * diag: Update log and event mask code ranges
  * usb: gadget: Clear string index for RMNET
  * Merge "drivers: usb: enable SET_REPORT through EP0 feature"
  * drivers: usb: enable SET_REPORT through EP0 feature
  * Merge "af_unix: fix garbage collect vs MSG_PEEK"
  * Merge "mmc: host: Add recovery_finish crypto vops"
  * Merge "dfc: hold wakelock while powersave timer is running"
  * Merge "uapi: sound: add bt external sink delay parameter"
  * Merge "msm: npu: remove asynchronous network execution support"
  * Merge "diag: Avoid possible out of bound access while sending masks"
  * Merge "diag: Sanitize non-hdlc pkt length against buffer capacity"
  * Merge "diag: Allow DCI packets to be copied separately"
  * Merge "clk: qcom: debugcc: Remove im_sleep/xo_div4 for QM215"
  * Merge "msm: gsi: Stop Channel support when in Flow Control State"
  * clk: qcom: debugcc: Remove gcc_pcnoc_mpu_cfg_ahb_clk for QM215
  * clk: qcom: debugcc: Remove im_sleep/xo_div4 for QM215
  * af_unix: fix garbage collect vs MSG_PEEK
  * net: split out functions related to registering inflight socket files
  * uapi: sound: add bt external sink delay parameter
  * mhi: core: Increase RDDM timeout to 350ms
  * msm: npu: remove asynchronous network execution support
  * usb: max-3421: Prevent corruption of freed memory
  * mmc: host: Add recovery_finish crypto vops
  * seq_file: disallow extremely large seq buffer allocations
  * dfc: hold wakelock while powersave timer is running
  * diag: Avoid possible out of bound access while sending masks
  * diag: Sanitize non-hdlc pkt length against buffer capacity
  * diag: Allow DCI packets to be copied separately
  * msm: gsi: Stop Channel support when in Flow Control State
  * msm: kgsl: Signal fence only if last fence refcount was not put
  * Merge "msm: adsprpc: Handle UAF in process shell memory"
  * media: v4l2: Allow ioctl type of "U" for video devices like usb camera
  * msm: adsprpc: Handle UAF in process shell memory
  * Merge commit '82146398a44b8081ef002068af683b1f44f4b226' into kernel.lnx.4.19.r23-rel
  * Merge "cnss2: Check if firmware asserts before power off for CBC"
  * Merge "net: qrtr: Use radix_tree_iter_delete to delete tx flow"
  * Merge "net: qrtr: Cleanup flow control during DEL proc"
  * cnss2: Check if firmware asserts before power off for CBC
  * Merge "net: qrtr: Cleanup flow control during remote socket release"
  * Merge "usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path"
  * usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path
  * net: qrtr: Use radix_tree_iter_delete to delete tx flow
  * net: qrtr: Cleanup flow control during DEL proc
  * net: qrtr: Cleanup flow control during remote socket release
  * net: qrtr: Converting DEL_PROC command to BYE command
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * defconfig: Enable Novatek NT36xxx Touch for tron target
  * cnss2: Update bound checks for sbl reg dumps to SRAM mem range

n.n.n / 2022-04-20
==================

  * version 7
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * sysfs: Add sysfs_emit and sysfs_emit_at to format sysfs output
  * Revert "Add toggle for disabling newly added USB devices"
  * cpufreq: schedutil: Use kobject release() method to free sugov_tunables
  * f2fs: avoid infinite loop to flush node pages
  * f2fs: fix wrong condition check when failing metapage read
  * f2fs: remove unnecessary f2fs_lock_op in f2fs_new_inode
  * f2fs: fix to do sanity check on .cp_pack_total_block_count
  * f2fs: compress: fix to print raw data size in error path of lz4 decompression
  * f2fs: remove redundant parameter judgment
  * drivers: input: aw8697: Fix missmerge
  * ARM64: dts: Use new awinic haptics configuration
  * input: misc: aw8697_haptic: Upgrade from zeus-s-oss
  * arch: dts: Throttle charging speed and Optimise charging current
  * msm: thermal: simple: account for battery temperature in calculations
  * arm64: lib: Memory utilities optimization
  * proc: do not let google camera run in the background
  * thermal: Increase thermal trip points to 16
  * f2fs: use spin_lock to avoid hang
  * f2fs: remove unnecessary read for F2FS_FITS_IN_INODE
  * f2fs: avoid an infinite loop in f2fs_sync_dirty_inodes
  * f2fs: fix to do sanity check on curseg->alloc_type
  * f2fs: fix to avoid potential deadlock
  * f2fs: fix missing free nid in f2fs_handle_failed_inode
  * f2fs: add a way to limit roll forward recovery time
  * f2fs: adjust readahead block number during recovery
  * f2fs: expose discard related parameters in sysfs
  * f2fs: move discard parameters into discard_cmd_control
  * f2fs: do not allow partial truncation on pinned file
  * version 6
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge d09499e9c8a99763bf9afa92c06601070523dc3e on remote branch
  * spi: spi-msm-geni: Put device to suspend if PM status is active
  * Merge "cfg80211: Add support to advertize OCV support"
  * Merge "spi: spi-msm-geni: Protect from register access in suspend state"
  * Merge "spi: spi-msm-geni: Keep device to suspend if PM call fails"
  * cfg80211: Add support to advertize OCV support
  * msm: kgsl: Reduce latency while processing ioctls
  * setlocalversion: Never append a "+" to localversion
  * drivers:block:zram:default lz4
  * zram: Move default compression algorithm choice to Kconfig
  * defconfig: enable lz4 compression
  * defconfigs: disable msm performance
  * USB: f_fs: Fix disconnect check during ongoing IO
  * Merge "msm: diag: fix copyright"
  * Merge "drivers: soc: qcom: Initialize blocking notifier as per lockdep"
  * msm: diag: fix copyright
  * Merge "diag: Validate the dci client before sending dci packet"
  * diag: Validate the dci client before sending dci packet
  * drivers: soc: qcom: Initialize blocking notifier as per lockdep
  * version 5
  * defconfig: regen
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * usb: gadget: u_audio: fix race condition on endpoint stop
  * usb: gadget: u_audio: Free requests only after callback
  * defconfig: Enable usb peripheral audio on SXR2130
  * Merge "diag: Ensure dci entry is valid before sending the packet"
  * Merge "msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()"
  * msm: ADSPRPC: Fix to avoid Use after free in fastrpc_init_process
  * Merge "msm: synx: fix copyright"
  * Merge "msm: ipa3: Fix to validate the NAT table entries during NAT table init"
  * msm: synx: fix copyright
  * clk: qcom: vdd-level: Update the vdd level for CX on Khaje
  * msm: ipa3: Fix to validate the NAT table entries during NAT table init
  * Merge 533e678218e41df69501158ff6eb84deafb6771c on remote branch
  * msm: synx: fix copyright
  * clk: qcom: gpucc: Update the vdd level for CX on Khaje
  * diag: Ensure dci entry is valid before sending the packet
  * Merge "usb: pd: Process request message as soon as it is received"
  * usb: pd: Process request message as soon as it is received
  * Merge "dfc: reset tx queue"
  * mmc: sdhci-msm: Update the MGPI SW check to avoid irq timeout
  * msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()
  * Merge "msm: ipa3: fix to cleanup the dma allocation"
  * Merge "msm: synx: remove synx handle details from logging"
  * dfc: reset tx queue
  * clk: qcom: gpucc: Add support for higher frequency for Khaje
  * Merge "clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje"
  * msm: ipa3: fix to cleanup the dma allocation
  * Merge "include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner"
  * clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje
  * Merge "msm: adsprpc: Handle UAF in fastrpc debugfs read"
  * include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner
  * Merge "usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind"
  * usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind
  * version 4
  * firmware: focaltech: Upgrade FW from haydin
  * touchscreen: focaltech_spi: Upgrade regardless of FW version check
  * techpack: audio: Mark msm-dai-q6 drivers as sync probe
  * techpack/audio: kona: Report correct key code of headset buttons
  * thermal: qcom: bcl: Return before thermal trip evaluates
  * defconfigs: enable Simple MSM Thermal
  * ARM64: dts: umi-sm8250: configure msm-thermal-simple
  * msm: thermal: simple: Introduce simple MSM thermal solution [msm-4.14]
  * version 3
  * drivers: thermal: limits-dcvs: Always build driver
  * drivers: power: supply: Disable battery capacity learning
  * defconfig: revert back to 100Hz tick
  * build: version 2
  * build: its Snow time !!!
  * drivers: usb: dwc3: Specify sync probe for dwc3-of-simple.
  * drivers: usb: dwc3: Specify sync probe for msm-dwc3 driver
  * drivers: usb: dwc3: Specify sync probe for dwc3 driver
  * drivers: usb: pd: remove fix pd output for 5v
  * Revert "usb: pd: Register typec partner in case AAA is connected"
  * PM / sleep: Skip OOM killer toggles when kernel is compiled for Android
  * drm: msm: optimize interpolation
  * techpack: drm: sde: fix a race condition
  * platform: msm: Fix dangerous relocation
  * defconfig: Bump SLMK minfree & timeout
  * power: process: Use lesser time to enter sleep
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * qos: Don't allow userspace to impose restrictions on CPU idle levels
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * dts: kona-gpu: Remove qos active latency node
  * treewide: use mi drm notifier
  * drivers: power: add timeouts to wakelocks
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * ARM64: umi: overlay gpu data from kona-v2-gpu.dtsi
  * arm64: dts: kona-v2-gpu: Overclock to 670mhz
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * gitignore: add out directory
  * add yarpiins build stuff
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/defconfig: apollo: Bringup custom defconfig
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * Revert "scsi: ufs: increase query timeout"
  * Revert "drivers: Add support for memory fragmentation test simulator driver"
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * ANDROID: sched: Exempt paused CPU from nohz idle balance
  * ARM64: configs: Make configs required to pass CTS tests mandatory
  * ARM64/defconfig: umi: Disable LSE atomics
  * ARM64/defconfig: umi: Use a timer frequency of 100 Hz
  * arm64: Inline the spin lock function family
  * ARM64/defconfig: umi: Disable unused errata
  * ARM64/defconfig: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * UPSTREAM: arm64: link with -z norelro for LLD or aarch64-elf
  * techpack: display: msm: dsi: set doze brightness for aod correctly
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64/defconfig: umi: Disable some unuse drivers
  * ARM64/defconfig: umi: Disable stability debug configs Disable the following stability debug configs for shipping: - CONFIG_EDAC_KRYO_ARM64_PANIC_ON_UE
  * ARM64/defconfig: umi: Enable LLD, RELR, Clang ThinLTO optimizations
  * ANDROID: sched: EAS: take cstate into account when selecting idle core
  * ARM64/defconfig: umi: Enable jump label
  * arm64/kernel: jump_label: Switch to relative references
  * locking/static_key: Add support for deferred static branches
  * jump_label: Fix NULL dereference bug in __jump_label_mod_update()
  * jump_label: Annotate entries that operate on __init code earlier
  * jump_label: Implement generic support for relative references
  * jump_label: Abstract jump_entry member accessors
  * jump_label: Use static_key_linked() accessor
  * mm/vmalloc.c: switch to WARN_ON() and move it under unlink_va()
  * mm/vmalloc.c: get rid of one single unlink_va() when merge
  * mm/vmalloc.c: preload a CPU with one object for split purpose
  * mm/vmalloc.c: remove "node" argument
  * mm/vmalloc.c: fix typo in comment
  * vmalloc: export __vmalloc_node_range for CONFIG_TEST_VMALLOC_MODULE
  * mm/vmalloc: pass VM_USERMAP flags directly to __vmalloc_node_range()
  * mm/vmalloc: do not call kmemleak_free() on not yet accounted memory
  * mm/vmalloc.c: make vmalloc_32_user() align base kernel virtual address to SHMLBA
  * mm: convert totalram_pages and totalhigh_pages variables to atomic
  * vfree: add debug might_sleep()
  * mm/vmalloc.c: improve vfree() kerneldoc
  * drivers: scsi: ufs: Remove unused d_lu_wb_buf_alloc
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64/defconfig: umi: Disable QHEE kernel memory protection
  * qcacld-3.0: Free a bunch of pkts at once
  * Makefile: Use O3 optimization level for Clang LTO
  * Makefile: Use -O3 optimization level
  * drivers: thermal: Don't qualify thermal polling as high priority
  * ARM64/defconfig: umi: Enable power efficient workqueues
  * ARM64/defconfig: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64/defconfig: Disable serial console support
  * arm64: Don't build legacy QCOM DTS.
  * rcu: fix a performance regression
  * qcacmn: Fix build error with !IPA_OFFLOAD
  * ARM64/defconfig: umi: Disable qti core control and sched autogroup
  * ARM: dts: kona: Disable IRQ debugging
  * ARM64/defconfig: umi: Disable some debug drivers
  * f2fs: Demote GC thread to idle scheduler class
  * f2fs: Set ioprio of GC kthread to idle
  * f2fs: Enlarge min_fsync_blocks to 20
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * UPSTREAM: zram: move backing_dev under macro CONFIG_ZRAM_WRITEBACK
  * UPSTREAM: zram: fix broken page writeback
  * UPSTREAM: zram: fix return value on writeback_store
  * UPSTREAM: zram: support page writeback
  * BACKPORT: zcomp: Use ARRAY_SIZE() for backends list
  * BACKPORT: zram: Allocate struct zcomp_strm as per-CPU memory
  * Revert "zram: introduce zram_entry to prepare dedup functionality"
  * Revert "zram: implement deduplication in zram"
  * Revert "zram: make deduplication feature optional"
  * Revert "zram: compare all the entries with same checksum for deduplication"
  * Revert "zram: fix race condition while returning zram_entry refcount"
  * ASoC: pcm: Add 24bit playback audio support
  * ARM64: configs: enable CONFIG_WIREGUARD
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * arm64/defconfig: umi: Enable crypto LZ4
  * arm64/defconfig: umi: Enable zram-writeback support
  * ARM64: umi/defconfigs: Bringup custom config
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
  * block: zram: Fix idle/writeback string compare
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * drivers: gpu: msm: Only build adreno 6xx part
  * video: backlight: disable modules enabled by default
  * media: gspca: disable gspca module
  * net: disable bridge netfilter module
  * net: ipv4: disable modules which are enabled by default
  * qcacmn: Fix suspicious string concatenation warning in fwlog
  * rmnet_shs: Fix CFI violation in packet assignment
  * rmnet_perf: Fix CFI violation in packet deaggregation
  * ntfs: Fix ntfs_test_inode and ntfs_init_locked_inode function type If the kernel is built with CFI we hit a __cfi_check_fail while mounting a partition
  * power: supply: Classify Battery Monitor Systems as batteries
  * cpufreq: stats: Replace the global lock with atomic.
  * techpack: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * techpack: camera: Unmap secure buffers in secure usecase
  * subsystem_restart: Always performs soft resets when subsystems crash
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * platform: msm: gsi: Export symbols only if compiled as module
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * scsi: ufs: Force enable write booster feature on UFS 3.1
  * scsi: ufs: Disable write booster feature support on UFS 2.2
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * treewide: Remove all Android.mk files
  * techpack: audio: Silence some logspam
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: Remove xiaomi early fingerprint wakeup optimization
  * audio: swr-mstr-ctrl: Fix unbalanced IRQ condition in swrm_runtime_suspend()
  * ARM64: configs: Enable XFRM_MIGRATE
  * ARM64: configs: Remove some debugging related features
  * ARM64: configs: Enable ARM64 Crypto Extensions SM3/4 and CRC32
  * ARM64: configs: Enable NTFS Filesystem
  * ARM64: configs: Enable ExFAT Filesystem
  * msm-poweroff: Store restart reason in panic
  * printk: Increase kernel log buffer size
  * smb5: Fix SDP current override for USB phy speed
  * mm: Disable watermark boost feature for K4.19
  * ARM64: configs: Skip file system sync in suspend
  * ARM64: configs: Disable PASR on kona devices
  * ARM64: configs: Reduce ION pool fill mark to 48MB
  * ARM64: configs: Build seperate DTB/DTBOs
  * ARM64: configs: Concatenate dtbs inside kernel image
  * ARM64: configs: Build Compressed Kernel Image
  * ARM64: configs: Build Qualcomm Atheros CLD WLAN module
  * ARM64: configs: Enable CPU frequency transition statistics
  * ARM64: configs: thyme: Enable support for block device writeback throttling
  * ARM64: configs: lmi: Enable AW2015 RGB LED support
  * ARM64: configs: lmi: Enable camera motor drivers
  * ARM64: configs: cas: Enable thermal dimming
  * ARM64: configs: apollo,cas: Enable panic on oops
  * ARM64: configs: apollo: Enable GPIO testing mode driver
  * ARM64: configs: Increase FRAME_WARN size to 4096
  * ARM64: configs: Enable Dynamic printk support
  * ARM64: configs: Disable MSM 11AD chip support
  * ARM64: configs: Enable xiaomi touchfeature driver
  * ARM64: configs: Disable USB CDC ADM support
  * ARM64: configs: Disable APP Armor security
  * ARM64: configs: Enable Ultrasound proximity driver
  * ARM64: configs: Enable USB Remote NDIS support
  * ARM64: configs: Disable USB Serial converter support
  * ARM64: configs: Disable V4L2 Videobuf2 Core
  * ARM64: configs: Enable SPI connected IR LED support
  * ARM64: configs: Enable PM8008 regulator driver
  * ARM64: configs: Enable device specific power supply drivers
  * ARM64: configs: Enable high performance logchar driver
  * ARM64: configs: Enable fastboot oem uart-enable command support
  * ARM64: configs: Disable support for Virtual terminal
  * ARM64: configs: Enable device specific fingerprint drivers
  * ARM64: configs: Enable AWINIC AW8697 haptics driver
  * ARM64: configs: Disable QTI Haptics support
  * ARM64: configs: Enable device specific touchscreen drivers
  * ARM64: configs: Disable Open-channel SSD target support
  * ARM64: configs: Enable CNSS QCA6390 chipset support
  * ARM64: configs: Enable QTI Smart Link Aggregation driver
  * ARM64: configs: Enable NAT, PEDIT and CSUM classifier Actions
  * ARM64: configs: Disable DEBUG_FS
  * ARM64: configs: Build Wilocity wil6210 driver
  * ARM64: configs: Enable Memory Technology Device (MTD) support
  * ARM64: configs: Disable Modules signature
  * ARM64: configs: Enable pstore support
  * ARM64: configs: Increase Kernel log buffer size
  * ARM64: configs: Enable Audit support
  * ARM64: configs: Enable device specific board platform
  * ARM64: configs: Generate xiaomi sm8250 devices configs from kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * bpf: Get the comm hash of the socket process stored inside sk buffer
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
  * fs: fuse: Implement FUSE passthrough
  * disp: msm: sde: Force SDE fd to start from 1
  * firmware: Import xiaomi touchscreens firmware
  * video: backlight: qcom-spmi-wled: Disable CABC on low brightness
  * video: backlight: Add support for thermal backlight dimming on J1S
  * video: backlight: Create brightness clone sysfs node
  * video: backlight: Notify on brightness node change
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * staging: android: ion: adjust system heap pool orders
  * staging: android: ion: Add xiaomi ion camera heap modifications
  * spi: gen-qcom: increase transfer timeout
  * soc: qcom: Add socinfo support for xiaomi sm8250 and sm7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
  * scsi: ufs: increase query timeout
  * mtd: devices: block2mtd: Sync scsi I/O on panic
  * debug: Add support to dump kmsg logs in kernel panic/oops using mtdoops
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * pinctrl: msm: Disable access to specified reserved gpios
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * cnss2: Add support for loading different bdwlan firmwares
  * net: cnss2: Add paramter to disable NV MAC support
  * net: cnss2: Increase cnss-daemon qmi timeout
  * media: rc: Add xiaomi modifications to IR driver
  * media: msm: cvp: Replace fence request handler kthread with workqueue
  * drivers: leds: Allow switching between dual flashlight leds
  * drivers: leds: Add support for AWINIC AW2015 3 Channel LED driver
  * drivers: cpuidle: Add xiaomi parameter to disable cpuidle and lpm
  * drivers: iio: Add support for Xiaomi ultrasound proximity sensor
  * drm: msm: Import xiaomi drm modifications
  * thermal: bcl_pmic5: Apply xiaomi thermal modifications
  * cpufreq: qcom: Always report maximum value of freq in limits_mitigation_notify()
  * drivers: thermal: Switch to CPUFREQ_ADJUST
  * mhi: mhi_qcom: Apply xiaomi modifications to sdx55 modem
  * drivers: misc: Import xiaomi GPIO testing mode driver
  * drivers: misc: Import Texas Instruments H-Bridge Stepper Motor Driver
  * drivers: misc: Import AKM Sensortecs AK09970 HALL sensor driver
  * block: Disable preemption before request_fn calls
  * ARM64: dts: qcom: Import kona mtp devicetree
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * input: touchscreen: Add support for xiaomi touchfeature dirver
  * input: touchscreen: Add support for Novatek NT36672c SPI touchscreen driver
  * input: touchscreen: Add support for GOODIX GT9886 touchscreen driver
  * input: touchscreen: Add support for STM FTS521 touchscreen driver
  * input: touchscreen: Add support for FocalTech ft8719/ft5452 touchscreen drivers
  * input: touchscreen: focaltech_spi: Don't ignore firmware files
  * input: touchscreen: Add support for FocalTech ft3658 touchscreen driver
  * input: touchscreen: Disable caf default touchscreen drivers
  * input: misc: aw8697_haptic: Add support to reload firmware for umi
  * input: misc: Add support for AWINIC AW8696 Haptic driver
  * input: fingerprint: Add support for GOODIX FOD fingerprint driver
  * input: fingerprint: Add support for GOODIX fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * drivers: Add support for memory fragmentation test simulator driver
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * techpack: audio: nuke xlogchar
  * Merge commit '99646a7c5d9584934073bd5c49438993f44c2f31' into android11-base
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * Add 'arch/arm64/boot/dts/vendor/' from commit 'bd2d6b0afa1f8aba19e41ce3bc29c16745595efa'
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * techpack: data: Build high performance ipa/rmnet drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * Revert "selinux: Relocate ss_initialized and selinux_enforcing to separate 4k"
  * ipa3: fix improper size checks
  * msm: ipa: Fix Makefile
  * kbuild: Remove gcc-wrapper
  * add toggle for disabling newly added USB devices
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * techpack/pinctrl-lpi: initialise at late_initcall
  * techpack: audio: makefile: do not export all the variables
  * techpack: audio: Correct symlinks
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into lineage-19.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/camera-kernel into 'LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-00300-XXXX.QSSI12.0
  * msm: adsprpc: Handle UAF in fastrpc debugfs read
  * usb: gadget: f_hid: Fix unbind issues
  * usb: gadget: f_uvc: Fix unbind issues
  * Merge "clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI"
  * msm: synx: remove synx handle details from logging
  * clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI
  * Merge "input: misc: qcom-power-on: Add support to log KPDPWR status"
  * Merge "msm: kgsl: Fix gpuaddr_in_range() to check upper bound"
  * mmc: sdhci-msm: Ensure SD card power isn't ON when card removed
  * Merge "clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL"
  * msm: kgsl: Fix gpuaddr_in_range() to check upper bound
  * Merge "usb: max-3421: Prevent corruption of freed memory"
  * clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL
  * spi: spi-msm-geni: Protect from register access in suspend state
  * spi: spi-msm-geni: Keep device to suspend if PM call fails
  * Merge "regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling"
  * Merge "seq_file: disallow extremely large seq buffer allocations"
  * regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling
  * Merge a425c3837db7a914ad21e6f5634b4c6dc7083b43 on remote branch
  * usb: gadget: f_uac1: Add support for UAC1 function
  * Merge "tzlog: update struct to get normal and fatal diag logs"
  * Merge "fs: crypto: Maintain reference count for class keys"
  * input: misc: qcom-power-on: Add support to log KPDPWR status
  * Merge "diag: Update log and event mask code ranges"
  * tzlog: update struct to get normal and fatal diag logs
  * firmware: qcom: Remove garbage characters from qsee log
  * firmware: qcom: add enlarged qsee log support
  * firmware: qcom: encrypted tz and qsee log support
  * fs: crypto: Maintain reference count for class keys
  * Merge "usb: pd: always log when state machine work is queued"
  * usb: pd: always log when state machine work is queued
  * leds: qti-tri-led: remove LED_KEEP_TRIGGER flag
  * usb: pd: add usbpd_dbg() in pd_send_msg()
  * Merge "mhi: core: Increase RDDM timeout to 350ms"
  * diag: Update log and event mask code ranges
  * usb: gadget: Clear string index for RMNET
  * Merge "drivers: usb: enable SET_REPORT through EP0 feature"
  * drivers: usb: enable SET_REPORT through EP0 feature
  * Merge "af_unix: fix garbage collect vs MSG_PEEK"
  * Merge "mmc: host: Add recovery_finish crypto vops"
  * Merge "dfc: hold wakelock while powersave timer is running"
  * Merge "uapi: sound: add bt external sink delay parameter"
  * Merge "msm: npu: remove asynchronous network execution support"
  * Merge "diag: Avoid possible out of bound access while sending masks"
  * Merge "diag: Sanitize non-hdlc pkt length against buffer capacity"
  * Merge "diag: Allow DCI packets to be copied separately"
  * Merge "clk: qcom: debugcc: Remove im_sleep/xo_div4 for QM215"
  * Merge "msm: gsi: Stop Channel support when in Flow Control State"
  * clk: qcom: debugcc: Remove gcc_pcnoc_mpu_cfg_ahb_clk for QM215
  * clk: qcom: debugcc: Remove im_sleep/xo_div4 for QM215
  * af_unix: fix garbage collect vs MSG_PEEK
  * net: split out functions related to registering inflight socket files
  * uapi: sound: add bt external sink delay parameter
  * mhi: core: Increase RDDM timeout to 350ms
  * msm: npu: remove asynchronous network execution support
  * usb: max-3421: Prevent corruption of freed memory
  * mmc: host: Add recovery_finish crypto vops
  * seq_file: disallow extremely large seq buffer allocations
  * dfc: hold wakelock while powersave timer is running
  * diag: Avoid possible out of bound access while sending masks
  * diag: Sanitize non-hdlc pkt length against buffer capacity
  * diag: Allow DCI packets to be copied separately
  * msm: gsi: Stop Channel support when in Flow Control State
  * msm: kgsl: Signal fence only if last fence refcount was not put
  * Merge "msm: adsprpc: Handle UAF in process shell memory"
  * media: v4l2: Allow ioctl type of "U" for video devices like usb camera
  * msm: adsprpc: Handle UAF in process shell memory
  * Merge commit '82146398a44b8081ef002068af683b1f44f4b226' into kernel.lnx.4.19.r23-rel
  * Merge "cnss2: Check if firmware asserts before power off for CBC"
  * Merge "net: qrtr: Use radix_tree_iter_delete to delete tx flow"
  * Merge "net: qrtr: Cleanup flow control during DEL proc"
  * cnss2: Check if firmware asserts before power off for CBC
  * Merge "net: qrtr: Cleanup flow control during remote socket release"
  * Merge "usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path"
  * usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path
  * net: qrtr: Use radix_tree_iter_delete to delete tx flow
  * net: qrtr: Cleanup flow control during DEL proc
  * net: qrtr: Cleanup flow control during remote socket release
  * net: qrtr: Converting DEL_PROC command to BYE command
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * defconfig: Enable Novatek NT36xxx Touch for tron target
  * cnss2: Update bound checks for sbl reg dumps to SRAM mem range

n.n.n / 2022-04-05
==================

  * cpufreq: schedutil: Use kobject release() method to free sugov_tunables
  * ANDROID: sched: fair: balance for single core cluster
  * drivers: power: supply: move everything to power efficient workqueues
  * drivers: power: supply: qcom: relax wakeup events and optimize
  * cpufreq: Don't let userspace set the max freq
  * cpufreq: Kill userspace CPU boosting entirely
  * f2fs: avoid infinite loop to flush node pages
  * f2fs: fix wrong condition check when failing metapage read
  * f2fs: remove unnecessary f2fs_lock_op in f2fs_new_inode
  * f2fs: fix to do sanity check on .cp_pack_total_block_count
  * f2fs: compress: fix to print raw data size in error path of lz4 decompression
  * f2fs: remove redundant parameter judgment
  * drivers: input: aw8697: Fix missmerge
  * ARM64: dts: Use new awinic haptics configuration
  * input: misc: aw8697_haptic: Upgrade from zeus-s-oss
  * arch: dts: Throttle charging speed and Optimise charging current
  * msm: thermal: simple: account for battery temperature in calculations
  * arm64: lib: Memory utilities optimization
  * proc: do not let google camera run in the background
  * sched,rt: Use cpumask_any*_distribute()
  * sched/core: Distribute tasks within affinity masks
  * power/wakelock: Add a timeout to wakelocks globally
  * thermal: Increase thermal trip points to 16
  * f2fs: use spin_lock to avoid hang
  * f2fs: remove unnecessary read for F2FS_FITS_IN_INODE
  * f2fs: avoid an infinite loop in f2fs_sync_dirty_inodes
  * f2fs: fix to do sanity check on curseg->alloc_type
  * f2fs: fix to avoid potential deadlock
  * f2fs: fix missing free nid in f2fs_handle_failed_inode
  * f2fs: add a way to limit roll forward recovery time
  * f2fs: adjust readahead block number during recovery
  * f2fs: expose discard related parameters in sysfs
  * f2fs: move discard parameters into discard_cmd_control
  * f2fs: do not allow partial truncation on pinned file
  * version 6
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge d09499e9c8a99763bf9afa92c06601070523dc3e on remote branch
  * spi: spi-msm-geni: Put device to suspend if PM status is active
  * Merge "cfg80211: Add support to advertize OCV support"
  * Merge "spi: spi-msm-geni: Protect from register access in suspend state"
  * Merge "spi: spi-msm-geni: Keep device to suspend if PM call fails"
  * cfg80211: Add support to advertize OCV support
  * msm: kgsl: Reduce latency while processing ioctls
  * setlocalversion: Never append a "+" to localversion
  * drivers:block:zram:default lz4
  * zram: Move default compression algorithm choice to Kconfig
  * defconfig: enable lz4 compression
  * defconfigs: disable msm performance
  * USB: f_fs: Fix disconnect check during ongoing IO
  * Merge "msm: diag: fix copyright"
  * Merge "drivers: soc: qcom: Initialize blocking notifier as per lockdep"
  * msm: diag: fix copyright
  * Merge "diag: Validate the dci client before sending dci packet"
  * diag: Validate the dci client before sending dci packet
  * drivers: soc: qcom: Initialize blocking notifier as per lockdep
  * version 5
  * defconfig: regen
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * usb: gadget: u_audio: fix race condition on endpoint stop
  * usb: gadget: u_audio: Free requests only after callback
  * defconfig: Enable usb peripheral audio on SXR2130
  * Merge "diag: Ensure dci entry is valid before sending the packet"
  * Merge "msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()"
  * msm: ADSPRPC: Fix to avoid Use after free in fastrpc_init_process
  * Merge "msm: synx: fix copyright"
  * Merge "msm: ipa3: Fix to validate the NAT table entries during NAT table init"
  * msm: synx: fix copyright
  * clk: qcom: vdd-level: Update the vdd level for CX on Khaje
  * msm: ipa3: Fix to validate the NAT table entries during NAT table init
  * Merge 533e678218e41df69501158ff6eb84deafb6771c on remote branch
  * msm: synx: fix copyright
  * clk: qcom: gpucc: Update the vdd level for CX on Khaje
  * diag: Ensure dci entry is valid before sending the packet
  * Merge "usb: pd: Process request message as soon as it is received"
  * usb: pd: Process request message as soon as it is received
  * Merge "dfc: reset tx queue"
  * mmc: sdhci-msm: Update the MGPI SW check to avoid irq timeout
  * msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()
  * Merge "msm: ipa3: fix to cleanup the dma allocation"
  * Merge "msm: synx: remove synx handle details from logging"
  * dfc: reset tx queue
  * clk: qcom: gpucc: Add support for higher frequency for Khaje
  * Merge "clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje"
  * msm: ipa3: fix to cleanup the dma allocation
  * Merge "include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner"
  * clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje
  * Merge "msm: adsprpc: Handle UAF in fastrpc debugfs read"
  * include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner
  * Merge "usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind"
  * usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind
  * version 4
  * firmware: focaltech: Upgrade FW from haydin
  * touchscreen: focaltech_spi: Upgrade regardless of FW version check
  * techpack: audio: Mark msm-dai-q6 drivers as sync probe
  * techpack/audio: kona: Report correct key code of headset buttons
  * thermal: qcom: bcl: Return before thermal trip evaluates
  * defconfigs: enable Simple MSM Thermal
  * ARM64: dts: umi-sm8250: configure msm-thermal-simple
  * msm: thermal: simple: Introduce simple MSM thermal solution [msm-4.14]
  * version 3
  * drivers: thermal: limits-dcvs: Always build driver
  * drivers: power: supply: Disable battery capacity learning
  * defconfig: revert back to 100Hz tick
  * build: version 2
  * build: its Snow time !!!
  * drivers: usb: dwc3: Specify sync probe for dwc3-of-simple.
  * drivers: usb: dwc3: Specify sync probe for msm-dwc3 driver
  * drivers: usb: dwc3: Specify sync probe for dwc3 driver
  * drivers: usb: pd: remove fix pd output for 5v
  * Revert "usb: pd: Register typec partner in case AAA is connected"
  * PM / sleep: Skip OOM killer toggles when kernel is compiled for Android
  * drm: msm: optimize interpolation
  * techpack: drm: sde: fix a race condition
  * platform: msm: Fix dangerous relocation
  * defconfig: Bump SLMK minfree & timeout
  * power: process: Use lesser time to enter sleep
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * qos: Don't allow userspace to impose restrictions on CPU idle levels
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * dts: kona-gpu: Remove qos active latency node
  * treewide: use mi drm notifier
  * drivers: power: add timeouts to wakelocks
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * ARM64: umi: overlay gpu data from kona-v2-gpu.dtsi
  * arm64: dts: kona-v2-gpu: Overclock to 670mhz
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * gitignore: add out directory
  * add yarpiins build stuff
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/defconfig: apollo: Bringup custom defconfig
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * Revert "scsi: ufs: increase query timeout"
  * Revert "drivers: Add support for memory fragmentation test simulator driver"
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * ANDROID: sched: Exempt paused CPU from nohz idle balance
  * ARM64: configs: Make configs required to pass CTS tests mandatory
  * ARM64/defconfig: umi: Disable LSE atomics
  * ARM64/defconfig: umi: Use a timer frequency of 100 Hz
  * arm64: Inline the spin lock function family
  * ARM64/defconfig: umi: Disable unused errata
  * ARM64/defconfig: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * UPSTREAM: arm64: link with -z norelro for LLD or aarch64-elf
  * techpack: display: msm: dsi: set doze brightness for aod correctly
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64/defconfig: umi: Disable some unuse drivers
  * ARM64/defconfig: umi: Disable stability debug configs Disable the following stability debug configs for shipping: - CONFIG_EDAC_KRYO_ARM64_PANIC_ON_UE
  * ARM64/defconfig: umi: Enable LLD, RELR, Clang ThinLTO optimizations
  * ANDROID: sched: EAS: take cstate into account when selecting idle core
  * ARM64/defconfig: umi: Enable jump label
  * arm64/kernel: jump_label: Switch to relative references
  * locking/static_key: Add support for deferred static branches
  * jump_label: Fix NULL dereference bug in __jump_label_mod_update()
  * jump_label: Annotate entries that operate on __init code earlier
  * jump_label: Implement generic support for relative references
  * jump_label: Abstract jump_entry member accessors
  * jump_label: Use static_key_linked() accessor
  * mm/vmalloc.c: switch to WARN_ON() and move it under unlink_va()
  * mm/vmalloc.c: get rid of one single unlink_va() when merge
  * mm/vmalloc.c: preload a CPU with one object for split purpose
  * mm/vmalloc.c: remove "node" argument
  * mm/vmalloc.c: fix typo in comment
  * vmalloc: export __vmalloc_node_range for CONFIG_TEST_VMALLOC_MODULE
  * mm/vmalloc: pass VM_USERMAP flags directly to __vmalloc_node_range()
  * mm/vmalloc: do not call kmemleak_free() on not yet accounted memory
  * mm/vmalloc.c: make vmalloc_32_user() align base kernel virtual address to SHMLBA
  * mm: convert totalram_pages and totalhigh_pages variables to atomic
  * vfree: add debug might_sleep()
  * mm/vmalloc.c: improve vfree() kerneldoc
  * drivers: scsi: ufs: Remove unused d_lu_wb_buf_alloc
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64/defconfig: umi: Disable QHEE kernel memory protection
  * qcacld-3.0: Free a bunch of pkts at once
  * Makefile: Use O3 optimization level for Clang LTO
  * Makefile: Use -O3 optimization level
  * drivers: thermal: Don't qualify thermal polling as high priority
  * ARM64/defconfig: umi: Enable power efficient workqueues
  * ARM64/defconfig: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64/defconfig: Disable serial console support
  * arm64: Don't build legacy QCOM DTS.
  * rcu: fix a performance regression
  * qcacmn: Fix build error with !IPA_OFFLOAD
  * ARM64/defconfig: umi: Disable qti core control and sched autogroup
  * ARM: dts: kona: Disable IRQ debugging
  * ARM64/defconfig: umi: Disable some debug drivers
  * f2fs: Demote GC thread to idle scheduler class
  * f2fs: Set ioprio of GC kthread to idle
  * f2fs: Enlarge min_fsync_blocks to 20
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * UPSTREAM: zram: move backing_dev under macro CONFIG_ZRAM_WRITEBACK
  * UPSTREAM: zram: fix broken page writeback
  * UPSTREAM: zram: fix return value on writeback_store
  * UPSTREAM: zram: support page writeback
  * BACKPORT: zcomp: Use ARRAY_SIZE() for backends list
  * BACKPORT: zram: Allocate struct zcomp_strm as per-CPU memory
  * Revert "zram: introduce zram_entry to prepare dedup functionality"
  * Revert "zram: implement deduplication in zram"
  * Revert "zram: make deduplication feature optional"
  * Revert "zram: compare all the entries with same checksum for deduplication"
  * Revert "zram: fix race condition while returning zram_entry refcount"
  * ASoC: pcm: Add 24bit playback audio support
  * ARM64: configs: enable CONFIG_WIREGUARD
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * arm64/defconfig: umi: Enable crypto LZ4
  * arm64/defconfig: umi: Enable zram-writeback support
  * ARM64: umi/defconfigs: Bringup custom config
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
  * block: zram: Fix idle/writeback string compare
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * drivers: gpu: msm: Only build adreno 6xx part
  * video: backlight: disable modules enabled by default
  * media: gspca: disable gspca module
  * net: disable bridge netfilter module
  * net: ipv4: disable modules which are enabled by default
  * qcacmn: Fix suspicious string concatenation warning in fwlog
  * rmnet_shs: Fix CFI violation in packet assignment
  * rmnet_perf: Fix CFI violation in packet deaggregation
  * ntfs: Fix ntfs_test_inode and ntfs_init_locked_inode function type If the kernel is built with CFI we hit a __cfi_check_fail while mounting a partition
  * power: supply: Classify Battery Monitor Systems as batteries
  * cpufreq: stats: Replace the global lock with atomic.
  * techpack: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * techpack: camera: Unmap secure buffers in secure usecase
  * subsystem_restart: Always performs soft resets when subsystems crash
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * platform: msm: gsi: Export symbols only if compiled as module
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * scsi: ufs: Force enable write booster feature on UFS 3.1
  * scsi: ufs: Disable write booster feature support on UFS 2.2
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * treewide: Remove all Android.mk files
  * techpack: audio: Silence some logspam
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: Remove xiaomi early fingerprint wakeup optimization
  * audio: swr-mstr-ctrl: Fix unbalanced IRQ condition in swrm_runtime_suspend()
  * ARM64: configs: Enable XFRM_MIGRATE
  * ARM64: configs: Remove some debugging related features
  * ARM64: configs: Enable ARM64 Crypto Extensions SM3/4 and CRC32
  * ARM64: configs: Enable NTFS Filesystem
  * ARM64: configs: Enable ExFAT Filesystem
  * msm-poweroff: Store restart reason in panic
  * printk: Increase kernel log buffer size
  * smb5: Fix SDP current override for USB phy speed
  * mm: Disable watermark boost feature for K4.19
  * ARM64: configs: Skip file system sync in suspend
  * ARM64: configs: Disable PASR on kona devices
  * ARM64: configs: Reduce ION pool fill mark to 48MB
  * ARM64: configs: Build seperate DTB/DTBOs
  * ARM64: configs: Concatenate dtbs inside kernel image
  * ARM64: configs: Build Compressed Kernel Image
  * ARM64: configs: Build Qualcomm Atheros CLD WLAN module
  * ARM64: configs: Enable CPU frequency transition statistics
  * ARM64: configs: thyme: Enable support for block device writeback throttling
  * ARM64: configs: lmi: Enable AW2015 RGB LED support
  * ARM64: configs: lmi: Enable camera motor drivers
  * ARM64: configs: cas: Enable thermal dimming
  * ARM64: configs: apollo,cas: Enable panic on oops
  * ARM64: configs: apollo: Enable GPIO testing mode driver
  * ARM64: configs: Increase FRAME_WARN size to 4096
  * ARM64: configs: Enable Dynamic printk support
  * ARM64: configs: Disable MSM 11AD chip support
  * ARM64: configs: Enable xiaomi touchfeature driver
  * ARM64: configs: Disable USB CDC ADM support
  * ARM64: configs: Disable APP Armor security
  * ARM64: configs: Enable Ultrasound proximity driver
  * ARM64: configs: Enable USB Remote NDIS support
  * ARM64: configs: Disable USB Serial converter support
  * ARM64: configs: Disable V4L2 Videobuf2 Core
  * ARM64: configs: Enable SPI connected IR LED support
  * ARM64: configs: Enable PM8008 regulator driver
  * ARM64: configs: Enable device specific power supply drivers
  * ARM64: configs: Enable high performance logchar driver
  * ARM64: configs: Enable fastboot oem uart-enable command support
  * ARM64: configs: Disable support for Virtual terminal
  * ARM64: configs: Enable device specific fingerprint drivers
  * ARM64: configs: Enable AWINIC AW8697 haptics driver
  * ARM64: configs: Disable QTI Haptics support
  * ARM64: configs: Enable device specific touchscreen drivers
  * ARM64: configs: Disable Open-channel SSD target support
  * ARM64: configs: Enable CNSS QCA6390 chipset support
  * ARM64: configs: Enable QTI Smart Link Aggregation driver
  * ARM64: configs: Enable NAT, PEDIT and CSUM classifier Actions
  * ARM64: configs: Disable DEBUG_FS
  * ARM64: configs: Build Wilocity wil6210 driver
  * ARM64: configs: Enable Memory Technology Device (MTD) support
  * ARM64: configs: Disable Modules signature
  * ARM64: configs: Enable pstore support
  * ARM64: configs: Increase Kernel log buffer size
  * ARM64: configs: Enable Audit support
  * ARM64: configs: Enable device specific board platform
  * ARM64: configs: Generate xiaomi sm8250 devices configs from kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * bpf: Get the comm hash of the socket process stored inside sk buffer
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
  * fs: fuse: Implement FUSE passthrough
  * disp: msm: sde: Force SDE fd to start from 1
  * firmware: Import xiaomi touchscreens firmware
  * video: backlight: qcom-spmi-wled: Disable CABC on low brightness
  * video: backlight: Add support for thermal backlight dimming on J1S
  * video: backlight: Create brightness clone sysfs node
  * video: backlight: Notify on brightness node change
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * staging: android: ion: adjust system heap pool orders
  * staging: android: ion: Add xiaomi ion camera heap modifications
  * spi: gen-qcom: increase transfer timeout
  * soc: qcom: Add socinfo support for xiaomi sm8250 and sm7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
  * scsi: ufs: increase query timeout
  * mtd: devices: block2mtd: Sync scsi I/O on panic
  * debug: Add support to dump kmsg logs in kernel panic/oops using mtdoops
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * pinctrl: msm: Disable access to specified reserved gpios
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * cnss2: Add support for loading different bdwlan firmwares
  * net: cnss2: Add paramter to disable NV MAC support
  * net: cnss2: Increase cnss-daemon qmi timeout
  * media: rc: Add xiaomi modifications to IR driver
  * media: msm: cvp: Replace fence request handler kthread with workqueue
  * drivers: leds: Allow switching between dual flashlight leds
  * drivers: leds: Add support for AWINIC AW2015 3 Channel LED driver
  * drivers: cpuidle: Add xiaomi parameter to disable cpuidle and lpm
  * drivers: iio: Add support for Xiaomi ultrasound proximity sensor
  * drm: msm: Import xiaomi drm modifications
  * thermal: bcl_pmic5: Apply xiaomi thermal modifications
  * cpufreq: qcom: Always report maximum value of freq in limits_mitigation_notify()
  * drivers: thermal: Switch to CPUFREQ_ADJUST
  * mhi: mhi_qcom: Apply xiaomi modifications to sdx55 modem
  * drivers: misc: Import xiaomi GPIO testing mode driver
  * drivers: misc: Import Texas Instruments H-Bridge Stepper Motor Driver
  * drivers: misc: Import AKM Sensortecs AK09970 HALL sensor driver
  * block: Disable preemption before request_fn calls
  * ARM64: dts: qcom: Import kona mtp devicetree
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * input: touchscreen: Add support for xiaomi touchfeature dirver
  * input: touchscreen: Add support for Novatek NT36672c SPI touchscreen driver
  * input: touchscreen: Add support for GOODIX GT9886 touchscreen driver
  * input: touchscreen: Add support for STM FTS521 touchscreen driver
  * input: touchscreen: Add support for FocalTech ft8719/ft5452 touchscreen drivers
  * input: touchscreen: focaltech_spi: Don't ignore firmware files
  * input: touchscreen: Add support for FocalTech ft3658 touchscreen driver
  * input: touchscreen: Disable caf default touchscreen drivers
  * input: misc: aw8697_haptic: Add support to reload firmware for umi
  * input: misc: Add support for AWINIC AW8696 Haptic driver
  * input: fingerprint: Add support for GOODIX FOD fingerprint driver
  * input: fingerprint: Add support for GOODIX fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * drivers: Add support for memory fragmentation test simulator driver
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * techpack: audio: nuke xlogchar
  * Merge commit '99646a7c5d9584934073bd5c49438993f44c2f31' into android11-base
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * Add 'arch/arm64/boot/dts/vendor/' from commit 'bd2d6b0afa1f8aba19e41ce3bc29c16745595efa'
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * techpack: data: Build high performance ipa/rmnet drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * Revert "selinux: Relocate ss_initialized and selinux_enforcing to separate 4k"
  * ipa3: fix improper size checks
  * msm: ipa: Fix Makefile
  * kbuild: Remove gcc-wrapper
  * add toggle for disabling newly added USB devices
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * techpack/pinctrl-lpi: initialise at late_initcall
  * techpack: audio: makefile: do not export all the variables
  * techpack: audio: Correct symlinks
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into lineage-19.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/camera-kernel into 'LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-00300-XXXX.QSSI12.0
  * msm: adsprpc: Handle UAF in fastrpc debugfs read
  * usb: gadget: f_hid: Fix unbind issues
  * usb: gadget: f_uvc: Fix unbind issues
  * Merge "clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI"
  * msm: synx: remove synx handle details from logging
  * clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI
  * Merge "input: misc: qcom-power-on: Add support to log KPDPWR status"
  * Merge "msm: kgsl: Fix gpuaddr_in_range() to check upper bound"
  * mmc: sdhci-msm: Ensure SD card power isn't ON when card removed
  * Merge "clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL"
  * msm: kgsl: Fix gpuaddr_in_range() to check upper bound
  * Merge "usb: max-3421: Prevent corruption of freed memory"
  * clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL
  * spi: spi-msm-geni: Protect from register access in suspend state
  * spi: spi-msm-geni: Keep device to suspend if PM call fails
  * Merge "regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling"
  * Merge "seq_file: disallow extremely large seq buffer allocations"
  * regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling
  * Merge a425c3837db7a914ad21e6f5634b4c6dc7083b43 on remote branch
  * usb: gadget: f_uac1: Add support for UAC1 function
  * Merge "tzlog: update struct to get normal and fatal diag logs"
  * Merge "fs: crypto: Maintain reference count for class keys"
  * input: misc: qcom-power-on: Add support to log KPDPWR status
  * Merge "diag: Update log and event mask code ranges"
  * tzlog: update struct to get normal and fatal diag logs
  * firmware: qcom: Remove garbage characters from qsee log
  * firmware: qcom: add enlarged qsee log support
  * firmware: qcom: encrypted tz and qsee log support
  * fs: crypto: Maintain reference count for class keys
  * Merge "usb: pd: always log when state machine work is queued"
  * usb: pd: always log when state machine work is queued
  * leds: qti-tri-led: remove LED_KEEP_TRIGGER flag
  * usb: pd: add usbpd_dbg() in pd_send_msg()
  * Merge "mhi: core: Increase RDDM timeout to 350ms"
  * diag: Update log and event mask code ranges
  * usb: gadget: Clear string index for RMNET
  * Merge "drivers: usb: enable SET_REPORT through EP0 feature"
  * drivers: usb: enable SET_REPORT through EP0 feature
  * Merge "af_unix: fix garbage collect vs MSG_PEEK"
  * Merge "mmc: host: Add recovery_finish crypto vops"
  * Merge "dfc: hold wakelock while powersave timer is running"
  * Merge "uapi: sound: add bt external sink delay parameter"
  * Merge "msm: npu: remove asynchronous network execution support"
  * Merge "diag: Avoid possible out of bound access while sending masks"
  * Merge "diag: Sanitize non-hdlc pkt length against buffer capacity"
  * Merge "diag: Allow DCI packets to be copied separately"
  * Merge "clk: qcom: debugcc: Remove im_sleep/xo_div4 for QM215"
  * Merge "msm: gsi: Stop Channel support when in Flow Control State"
  * clk: qcom: debugcc: Remove gcc_pcnoc_mpu_cfg_ahb_clk for QM215
  * clk: qcom: debugcc: Remove im_sleep/xo_div4 for QM215
  * af_unix: fix garbage collect vs MSG_PEEK
  * net: split out functions related to registering inflight socket files
  * uapi: sound: add bt external sink delay parameter
  * mhi: core: Increase RDDM timeout to 350ms
  * msm: npu: remove asynchronous network execution support
  * usb: max-3421: Prevent corruption of freed memory
  * mmc: host: Add recovery_finish crypto vops
  * seq_file: disallow extremely large seq buffer allocations
  * dfc: hold wakelock while powersave timer is running
  * diag: Avoid possible out of bound access while sending masks
  * diag: Sanitize non-hdlc pkt length against buffer capacity
  * diag: Allow DCI packets to be copied separately
  * msm: gsi: Stop Channel support when in Flow Control State
  * msm: kgsl: Signal fence only if last fence refcount was not put
  * Merge "msm: adsprpc: Handle UAF in process shell memory"
  * media: v4l2: Allow ioctl type of "U" for video devices like usb camera
  * msm: adsprpc: Handle UAF in process shell memory
  * Merge commit '82146398a44b8081ef002068af683b1f44f4b226' into kernel.lnx.4.19.r23-rel
  * Merge "cnss2: Check if firmware asserts before power off for CBC"
  * Merge "net: qrtr: Use radix_tree_iter_delete to delete tx flow"
  * Merge "net: qrtr: Cleanup flow control during DEL proc"
  * cnss2: Check if firmware asserts before power off for CBC
  * Merge "net: qrtr: Cleanup flow control during remote socket release"
  * Merge "usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path"
  * usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path
  * net: qrtr: Use radix_tree_iter_delete to delete tx flow
  * net: qrtr: Cleanup flow control during DEL proc
  * net: qrtr: Cleanup flow control during remote socket release
  * net: qrtr: Converting DEL_PROC command to BYE command
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * defconfig: Enable Novatek NT36xxx Touch for tron target
  * cnss2: Update bound checks for sbl reg dumps to SRAM mem range

n.n.n / 2022-03-02
==================

  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00700-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge d09499e9c8a99763bf9afa92c06601070523dc3e on remote branch
  * spi: spi-msm-geni: Put device to suspend if PM status is active
  * Merge "cfg80211: Add support to advertize OCV support"
  * Merge "spi: spi-msm-geni: Protect from register access in suspend state"
  * Merge "spi: spi-msm-geni: Keep device to suspend if PM call fails"
  * cfg80211: Add support to advertize OCV support
  * msm: kgsl: Reduce latency while processing ioctls
  * setlocalversion: Never append a "+" to localversion
  * drivers:block:zram:default lz4
  * zram: Move default compression algorithm choice to Kconfig
  * defconfig: enable lz4 compression
  * defconfigs: disable msm performance
  * USB: f_fs: Fix disconnect check during ongoing IO
  * Merge "msm: diag: fix copyright"
  * Merge "drivers: soc: qcom: Initialize blocking notifier as per lockdep"
  * msm: diag: fix copyright
  * Merge "diag: Validate the dci client before sending dci packet"
  * diag: Validate the dci client before sending dci packet
  * drivers: soc: qcom: Initialize blocking notifier as per lockdep
  * version 5
  * defconfig: regen
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * usb: gadget: u_audio: fix race condition on endpoint stop
  * usb: gadget: u_audio: Free requests only after callback
  * defconfig: Enable usb peripheral audio on SXR2130
  * Merge "diag: Ensure dci entry is valid before sending the packet"
  * Merge "msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()"
  * msm: ADSPRPC: Fix to avoid Use after free in fastrpc_init_process
  * Merge "msm: synx: fix copyright"
  * Merge "msm: ipa3: Fix to validate the NAT table entries during NAT table init"
  * msm: synx: fix copyright
  * clk: qcom: vdd-level: Update the vdd level for CX on Khaje
  * msm: ipa3: Fix to validate the NAT table entries during NAT table init
  * Merge 533e678218e41df69501158ff6eb84deafb6771c on remote branch
  * msm: synx: fix copyright
  * clk: qcom: gpucc: Update the vdd level for CX on Khaje
  * diag: Ensure dci entry is valid before sending the packet
  * Merge "usb: pd: Process request message as soon as it is received"
  * usb: pd: Process request message as soon as it is received
  * Merge "dfc: reset tx queue"
  * mmc: sdhci-msm: Update the MGPI SW check to avoid irq timeout
  * msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()
  * Merge "msm: ipa3: fix to cleanup the dma allocation"
  * Merge "msm: synx: remove synx handle details from logging"
  * dfc: reset tx queue
  * clk: qcom: gpucc: Add support for higher frequency for Khaje
  * Merge "clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje"
  * msm: ipa3: fix to cleanup the dma allocation
  * Merge "include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner"
  * clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje
  * Merge "msm: adsprpc: Handle UAF in fastrpc debugfs read"
  * include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner
  * Merge "usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind"
  * usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind
  * msm: adsprpc: Handle UAF in fastrpc debugfs read
  * usb: gadget: f_hid: Fix unbind issues
  * usb: gadget: f_uvc: Fix unbind issues
  * Merge "clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI"
  * msm: synx: remove synx handle details from logging
  * clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI
  * Merge "input: misc: qcom-power-on: Add support to log KPDPWR status"
  * Merge "msm: kgsl: Fix gpuaddr_in_range() to check upper bound"
  * mmc: sdhci-msm: Ensure SD card power isn't ON when card removed
  * Merge "clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL"
  * msm: kgsl: Fix gpuaddr_in_range() to check upper bound
  * Merge "usb: max-3421: Prevent corruption of freed memory"
  * clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL
  * spi: spi-msm-geni: Protect from register access in suspend state
  * spi: spi-msm-geni: Keep device to suspend if PM call fails
  * Merge "regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling"
  * Merge "seq_file: disallow extremely large seq buffer allocations"
  * regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling
  * usb: gadget: f_uac1: Add support for UAC1 function
  * input: misc: qcom-power-on: Add support to log KPDPWR status
  * usb: max-3421: Prevent corruption of freed memory
  * seq_file: disallow extremely large seq buffer allocations

n.n.n / 2022-02-01
==================

  * defconfig: regen
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00600-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * msm: synx: fix copyright
  * Merge 533e678218e41df69501158ff6eb84deafb6771c on remote branch
  * Merge "msm: ipa3: fix to cleanup the dma allocation"
  * Merge "msm: synx: remove synx handle details from logging"
  * clk: qcom: gpucc: Add support for higher frequency for Khaje
  * Merge "clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje"
  * msm: ipa3: fix to cleanup the dma allocation
  * Merge "include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner"
  * clk: qcom: vdd-level: Update the VDD levels for Bengal/Khaje
  * Merge "msm: adsprpc: Handle UAF in fastrpc debugfs read"
  * include: qcom,rpm-smd-regulator.h: Add SUPER_TURBO corner
  * Merge "usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind"
  * usb: gadget: qdss: Don't clear debug_inface_enabled upon unbind
  * msm: adsprpc: Handle UAF in fastrpc debugfs read
  * usb: gadget: f_hid: Fix unbind issues
  * usb: gadget: f_uvc: Fix unbind issues
  * Merge "clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI"
  * msm: synx: remove synx handle details from logging
  * clocksource/drivers/arm_arch_timer: Correct fault programming of CNTKCTL_EL1.EVNTI
  * Merge "input: misc: qcom-power-on: Add support to log KPDPWR status"
  * Merge "msm: kgsl: Fix gpuaddr_in_range() to check upper bound"
  * mmc: sdhci-msm: Ensure SD card power isn't ON when card removed
  * Merge "clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL"
  * msm: kgsl: Fix gpuaddr_in_range() to check upper bound
  * Merge "usb: max-3421: Prevent corruption of freed memory"
  * clk: qcom: clk-alpha-pll: Update the steps to slew the Lucid PLL
  * Merge "regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling"
  * Merge "seq_file: disallow extremely large seq buffer allocations"
  * regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling
  * usb: gadget: f_uac1: Add support for UAC1 function
  * input: misc: qcom-power-on: Add support to log KPDPWR status
  * usb: max-3421: Prevent corruption of freed memory
  * seq_file: disallow extremely large seq buffer allocations

n.n.n / 2022-01-08
==================

  * firmware: focaltech: Upgrade FW from haydin
  * touchscreen: focaltech_spi: Upgrade regardless of FW version check
  * techpack: audio: Mark msm-dai-q6 drivers as sync probe
  * techpack/audio: kona: Report correct key code of headset buttons
  * thermal: qcom: bcl: Return before thermal trip evaluates
  * defconfigs: enable Simple MSM Thermal
  * ARM64: dts: umi-sm8250: configure msm-thermal-simple
  * msm: thermal: simple: Introduce simple MSM thermal solution [msm-4.14]
  * version 3
  * drivers: thermal: limits-dcvs: Always build driver
  * drivers: power: supply: Disable battery capacity learning
  * defconfig: revert back to 100Hz tick
  * build: version 2
  * build: its Snow time !!!
  * drivers: usb: dwc3: Specify sync probe for dwc3-of-simple.
  * drivers: usb: dwc3: Specify sync probe for msm-dwc3 driver
  * drivers: usb: dwc3: Specify sync probe for dwc3 driver
  * drivers: usb: pd: remove fix pd output for 5v
  * Revert "usb: pd: Register typec partner in case AAA is connected"
  * PM / sleep: Skip OOM killer toggles when kernel is compiled for Android
  * drm: msm: optimize interpolation
  * techpack: drm: sde: fix a race condition
  * platform: msm: Fix dangerous relocation
  * defconfig: Bump SLMK minfree & timeout
  * power: process: Use lesser time to enter sleep
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * qos: Don't allow userspace to impose restrictions on CPU idle levels
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * dts: kona-gpu: Remove qos active latency node
  * treewide: use mi drm notifier
  * drivers: power: add timeouts to wakelocks
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * ARM64: umi: overlay gpu data from kona-v2-gpu.dtsi
  * arm64: dts: kona-v2-gpu: Overclock to 670mhz
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * gitignore: add out directory
  * add yarpiins build stuff
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/defconfig: apollo: Bringup custom defconfig
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * Revert "scsi: ufs: increase query timeout"
  * Revert "drivers: Add support for memory fragmentation test simulator driver"
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * ANDROID: sched: Exempt paused CPU from nohz idle balance
  * ARM64: configs: Make configs required to pass CTS tests mandatory
  * ARM64/defconfig: umi: Disable LSE atomics
  * ARM64/defconfig: umi: Use a timer frequency of 100 Hz
  * arm64: Inline the spin lock function family
  * ARM64/defconfig: umi: Disable unused errata
  * ARM64/defconfig: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * UPSTREAM: arm64: link with -z norelro for LLD or aarch64-elf
  * techpack: display: msm: dsi: set doze brightness for aod correctly
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64/defconfig: umi: Disable some unuse drivers
  * ARM64/defconfig: umi: Disable stability debug configs Disable the following stability debug configs for shipping: - CONFIG_EDAC_KRYO_ARM64_PANIC_ON_UE
  * ARM64/defconfig: umi: Enable LLD, RELR, Clang ThinLTO optimizations
  * ANDROID: sched: EAS: take cstate into account when selecting idle core
  * ARM64/defconfig: umi: Enable jump label
  * arm64/kernel: jump_label: Switch to relative references
  * locking/static_key: Add support for deferred static branches
  * jump_label: Fix NULL dereference bug in __jump_label_mod_update()
  * jump_label: Annotate entries that operate on __init code earlier
  * jump_label: Implement generic support for relative references
  * jump_label: Abstract jump_entry member accessors
  * jump_label: Use static_key_linked() accessor
  * mm/vmalloc.c: switch to WARN_ON() and move it under unlink_va()
  * mm/vmalloc.c: get rid of one single unlink_va() when merge
  * mm/vmalloc.c: preload a CPU with one object for split purpose
  * mm/vmalloc.c: remove "node" argument
  * mm/vmalloc.c: fix typo in comment
  * vmalloc: export __vmalloc_node_range for CONFIG_TEST_VMALLOC_MODULE
  * mm/vmalloc: pass VM_USERMAP flags directly to __vmalloc_node_range()
  * mm/vmalloc: do not call kmemleak_free() on not yet accounted memory
  * mm/vmalloc.c: make vmalloc_32_user() align base kernel virtual address to SHMLBA
  * mm: convert totalram_pages and totalhigh_pages variables to atomic
  * vfree: add debug might_sleep()
  * mm/vmalloc.c: improve vfree() kerneldoc
  * drivers: scsi: ufs: Remove unused d_lu_wb_buf_alloc
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64/defconfig: umi: Disable QHEE kernel memory protection
  * qcacld-3.0: Free a bunch of pkts at once
  * Makefile: Use O3 optimization level for Clang LTO
  * Makefile: Use -O3 optimization level
  * drivers: thermal: Don't qualify thermal polling as high priority
  * ARM64/defconfig: umi: Enable power efficient workqueues
  * ARM64/defconfig: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64/defconfig: Disable serial console support
  * arm64: Don't build legacy QCOM DTS.
  * rcu: fix a performance regression
  * qcacmn: Fix build error with !IPA_OFFLOAD
  * ARM64/defconfig: umi: Disable qti core control and sched autogroup
  * ARM: dts: kona: Disable IRQ debugging
  * ARM64/defconfig: umi: Disable some debug drivers
  * f2fs: Demote GC thread to idle scheduler class
  * f2fs: Set ioprio of GC kthread to idle
  * f2fs: Enlarge min_fsync_blocks to 20
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * UPSTREAM: zram: move backing_dev under macro CONFIG_ZRAM_WRITEBACK
  * UPSTREAM: zram: fix broken page writeback
  * UPSTREAM: zram: fix return value on writeback_store
  * UPSTREAM: zram: support page writeback
  * BACKPORT: zcomp: Use ARRAY_SIZE() for backends list
  * BACKPORT: zram: Allocate struct zcomp_strm as per-CPU memory
  * Revert "zram: introduce zram_entry to prepare dedup functionality"
  * Revert "zram: implement deduplication in zram"
  * Revert "zram: make deduplication feature optional"
  * Revert "zram: compare all the entries with same checksum for deduplication"
  * Revert "zram: fix race condition while returning zram_entry refcount"
  * ASoC: pcm: Add 24bit playback audio support
  * ARM64: configs: enable CONFIG_WIREGUARD
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * arm64/defconfig: umi: Enable crypto LZ4
  * arm64/defconfig: umi: Enable zram-writeback support
  * ARM64: umi/defconfigs: Bringup custom config
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
  * block: zram: Fix idle/writeback string compare
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * drivers: gpu: msm: Only build adreno 6xx part
  * video: backlight: disable modules enabled by default
  * media: gspca: disable gspca module
  * net: disable bridge netfilter module
  * net: ipv4: disable modules which are enabled by default
  * qcacmn: Fix suspicious string concatenation warning in fwlog
  * rmnet_shs: Fix CFI violation in packet assignment
  * rmnet_perf: Fix CFI violation in packet deaggregation
  * ntfs: Fix ntfs_test_inode and ntfs_init_locked_inode function type If the kernel is built with CFI we hit a __cfi_check_fail while mounting a partition
  * power: supply: Classify Battery Monitor Systems as batteries
  * cpufreq: stats: Replace the global lock with atomic.
  * techpack: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * techpack: camera: Unmap secure buffers in secure usecase
  * subsystem_restart: Always performs soft resets when subsystems crash
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * platform: msm: gsi: Export symbols only if compiled as module
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * scsi: ufs: Force enable write booster feature on UFS 3.1
  * scsi: ufs: Disable write booster feature support on UFS 2.2
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * treewide: Remove all Android.mk files
  * techpack: audio: Silence some logspam
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: Remove xiaomi early fingerprint wakeup optimization
  * audio: swr-mstr-ctrl: Fix unbalanced IRQ condition in swrm_runtime_suspend()
  * ARM64: configs: Enable XFRM_MIGRATE
  * ARM64: configs: Remove some debugging related features
  * ARM64: configs: Enable ARM64 Crypto Extensions SM3/4 and CRC32
  * ARM64: configs: Enable NTFS Filesystem
  * ARM64: configs: Enable ExFAT Filesystem
  * msm-poweroff: Store restart reason in panic
  * printk: Increase kernel log buffer size
  * smb5: Fix SDP current override for USB phy speed
  * mm: Disable watermark boost feature for K4.19
  * ARM64: configs: Skip file system sync in suspend
  * ARM64: configs: Disable PASR on kona devices
  * ARM64: configs: Reduce ION pool fill mark to 48MB
  * ARM64: configs: Build seperate DTB/DTBOs
  * ARM64: configs: Concatenate dtbs inside kernel image
  * ARM64: configs: Build Compressed Kernel Image
  * ARM64: configs: Build Qualcomm Atheros CLD WLAN module
  * ARM64: configs: Enable CPU frequency transition statistics
  * ARM64: configs: thyme: Enable support for block device writeback throttling
  * ARM64: configs: lmi: Enable AW2015 RGB LED support
  * ARM64: configs: lmi: Enable camera motor drivers
  * ARM64: configs: cas: Enable thermal dimming
  * ARM64: configs: apollo,cas: Enable panic on oops
  * ARM64: configs: apollo: Enable GPIO testing mode driver
  * ARM64: configs: Increase FRAME_WARN size to 4096
  * ARM64: configs: Enable Dynamic printk support
  * ARM64: configs: Disable MSM 11AD chip support
  * ARM64: configs: Enable xiaomi touchfeature driver
  * ARM64: configs: Disable USB CDC ADM support
  * ARM64: configs: Disable APP Armor security
  * ARM64: configs: Enable Ultrasound proximity driver
  * ARM64: configs: Enable USB Remote NDIS support
  * ARM64: configs: Disable USB Serial converter support
  * ARM64: configs: Disable V4L2 Videobuf2 Core
  * ARM64: configs: Enable SPI connected IR LED support
  * ARM64: configs: Enable PM8008 regulator driver
  * ARM64: configs: Enable device specific power supply drivers
  * ARM64: configs: Enable high performance logchar driver
  * ARM64: configs: Enable fastboot oem uart-enable command support
  * ARM64: configs: Disable support for Virtual terminal
  * ARM64: configs: Enable device specific fingerprint drivers
  * ARM64: configs: Enable AWINIC AW8697 haptics driver
  * ARM64: configs: Disable QTI Haptics support
  * ARM64: configs: Enable device specific touchscreen drivers
  * ARM64: configs: Disable Open-channel SSD target support
  * ARM64: configs: Enable CNSS QCA6390 chipset support
  * ARM64: configs: Enable QTI Smart Link Aggregation driver
  * ARM64: configs: Enable NAT, PEDIT and CSUM classifier Actions
  * ARM64: configs: Disable DEBUG_FS
  * ARM64: configs: Build Wilocity wil6210 driver
  * ARM64: configs: Enable Memory Technology Device (MTD) support
  * ARM64: configs: Disable Modules signature
  * ARM64: configs: Enable pstore support
  * ARM64: configs: Increase Kernel log buffer size
  * ARM64: configs: Enable Audit support
  * ARM64: configs: Enable device specific board platform
  * ARM64: configs: Generate xiaomi sm8250 devices configs from kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * bpf: Get the comm hash of the socket process stored inside sk buffer
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
  * fs: fuse: Implement FUSE passthrough
  * disp: msm: sde: Force SDE fd to start from 1
  * firmware: Import xiaomi touchscreens firmware
  * video: backlight: qcom-spmi-wled: Disable CABC on low brightness
  * video: backlight: Add support for thermal backlight dimming on J1S
  * video: backlight: Create brightness clone sysfs node
  * video: backlight: Notify on brightness node change
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * staging: android: ion: adjust system heap pool orders
  * staging: android: ion: Add xiaomi ion camera heap modifications
  * spi: gen-qcom: increase transfer timeout
  * soc: qcom: Add socinfo support for xiaomi sm8250 and sm7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
  * scsi: ufs: increase query timeout
  * mtd: devices: block2mtd: Sync scsi I/O on panic
  * debug: Add support to dump kmsg logs in kernel panic/oops using mtdoops
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * pinctrl: msm: Disable access to specified reserved gpios
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * cnss2: Add support for loading different bdwlan firmwares
  * net: cnss2: Add paramter to disable NV MAC support
  * net: cnss2: Increase cnss-daemon qmi timeout
  * media: rc: Add xiaomi modifications to IR driver
  * media: msm: cvp: Replace fence request handler kthread with workqueue
  * drivers: leds: Allow switching between dual flashlight leds
  * drivers: leds: Add support for AWINIC AW2015 3 Channel LED driver
  * drivers: cpuidle: Add xiaomi parameter to disable cpuidle and lpm
  * drivers: iio: Add support for Xiaomi ultrasound proximity sensor
  * drm: msm: Import xiaomi drm modifications
  * thermal: bcl_pmic5: Apply xiaomi thermal modifications
  * cpufreq: qcom: Always report maximum value of freq in limits_mitigation_notify()
  * drivers: thermal: Switch to CPUFREQ_ADJUST
  * mhi: mhi_qcom: Apply xiaomi modifications to sdx55 modem
  * drivers: misc: Import xiaomi GPIO testing mode driver
  * drivers: misc: Import Texas Instruments H-Bridge Stepper Motor Driver
  * drivers: misc: Import AKM Sensortecs AK09970 HALL sensor driver
  * block: Disable preemption before request_fn calls
  * ARM64: dts: qcom: Import kona mtp devicetree
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * input: touchscreen: Add support for xiaomi touchfeature dirver
  * input: touchscreen: Add support for Novatek NT36672c SPI touchscreen driver
  * input: touchscreen: Add support for GOODIX GT9886 touchscreen driver
  * input: touchscreen: Add support for STM FTS521 touchscreen driver
  * input: touchscreen: Add support for FocalTech ft8719/ft5452 touchscreen drivers
  * input: touchscreen: focaltech_spi: Don't ignore firmware files
  * input: touchscreen: Add support for FocalTech ft3658 touchscreen driver
  * input: touchscreen: Disable caf default touchscreen drivers
  * input: misc: aw8697_haptic: Add support to reload firmware for umi
  * input: misc: Add support for AWINIC AW8696 Haptic driver
  * input: fingerprint: Add support for GOODIX FOD fingerprint driver
  * input: fingerprint: Add support for GOODIX fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * drivers: Add support for memory fragmentation test simulator driver
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * techpack: audio: nuke xlogchar
  * Merge commit '99646a7c5d9584934073bd5c49438993f44c2f31' into android11-base
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * Add 'arch/arm64/boot/dts/vendor/' from commit 'bd2d6b0afa1f8aba19e41ce3bc29c16745595efa'
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * techpack: data: Build high performance ipa/rmnet drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * Revert "selinux: Relocate ss_initialized and selinux_enforcing to separate 4k"
  * ipa3: fix improper size checks
  * msm: ipa: Fix Makefile
  * kbuild: Remove gcc-wrapper
  * add toggle for disabling newly added USB devices
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * techpack/pinctrl-lpi: initialise at late_initcall
  * techpack: audio: makefile: do not export all the variables
  * techpack: audio: Correct symlinks
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into lineage-19.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/camera-kernel into 'LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.1.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.1.r1-00300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.1.r1-00300-XXXX.QSSI12.0
  * Merge a425c3837db7a914ad21e6f5634b4c6dc7083b43 on remote branch
  * Merge "tzlog: update struct to get normal and fatal diag logs"
  * Merge "fs: crypto: Maintain reference count for class keys"
  * Merge "diag: Update log and event mask code ranges"
  * tzlog: update struct to get normal and fatal diag logs
  * firmware: qcom: Remove garbage characters from qsee log
  * firmware: qcom: add enlarged qsee log support
  * firmware: qcom: encrypted tz and qsee log support
  * fs: crypto: Maintain reference count for class keys
  * Merge "usb: pd: always log when state machine work is queued"
  * usb: pd: always log when state machine work is queued
  * leds: qti-tri-led: remove LED_KEEP_TRIGGER flag
  * usb: pd: add usbpd_dbg() in pd_send_msg()
  * Merge "mhi: core: Increase RDDM timeout to 350ms"
  * diag: Update log and event mask code ranges
  * usb: gadget: Clear string index for RMNET
  * Merge "drivers: usb: enable SET_REPORT through EP0 feature"
  * drivers: usb: enable SET_REPORT through EP0 feature
  * Merge "af_unix: fix garbage collect vs MSG_PEEK"
  * Merge "mmc: host: Add recovery_finish crypto vops"
  * Merge "dfc: hold wakelock while powersave timer is running"
  * Merge "uapi: sound: add bt external sink delay parameter"
  * Merge "msm: npu: remove asynchronous network execution support"
  * Merge "diag: Avoid possible out of bound access while sending masks"
  * Merge "diag: Sanitize non-hdlc pkt length against buffer capacity"
  * Merge "diag: Allow DCI packets to be copied separately"
  * Merge "clk: qcom: debugcc: Remove im_sleep/xo_div4 for QM215"
  * Merge "msm: gsi: Stop Channel support when in Flow Control State"
  * clk: qcom: debugcc: Remove gcc_pcnoc_mpu_cfg_ahb_clk for QM215
  * clk: qcom: debugcc: Remove im_sleep/xo_div4 for QM215
  * af_unix: fix garbage collect vs MSG_PEEK
  * net: split out functions related to registering inflight socket files
  * uapi: sound: add bt external sink delay parameter
  * mhi: core: Increase RDDM timeout to 350ms
  * msm: npu: remove asynchronous network execution support
  * mmc: host: Add recovery_finish crypto vops
  * dfc: hold wakelock while powersave timer is running
  * diag: Avoid possible out of bound access while sending masks
  * diag: Sanitize non-hdlc pkt length against buffer capacity
  * diag: Allow DCI packets to be copied separately
  * msm: gsi: Stop Channel support when in Flow Control State
  * msm: kgsl: Signal fence only if last fence refcount was not put
  * Merge "msm: adsprpc: Handle UAF in process shell memory"
  * media: v4l2: Allow ioctl type of "U" for video devices like usb camera
  * msm: adsprpc: Handle UAF in process shell memory
  * Merge commit '82146398a44b8081ef002068af683b1f44f4b226' into kernel.lnx.4.19.r23-rel
  * Merge "cnss2: Check if firmware asserts before power off for CBC"
  * Merge "net: qrtr: Use radix_tree_iter_delete to delete tx flow"
  * Merge "net: qrtr: Cleanup flow control during DEL proc"
  * cnss2: Check if firmware asserts before power off for CBC
  * Merge "net: qrtr: Cleanup flow control during remote socket release"
  * Merge "usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path"
  * usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path
  * net: qrtr: Use radix_tree_iter_delete to delete tx flow
  * net: qrtr: Cleanup flow control during DEL proc
  * net: qrtr: Cleanup flow control during remote socket release
  * net: qrtr: Converting DEL_PROC command to BYE command
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * defconfig: Enable Novatek NT36xxx Touch for tron target
  * cnss2: Update bound checks for sbl reg dumps to SRAM mem range

n.n.n / 2021-12-13
==================

  * Merge tag 'LA.UM.9.12.r1-13500-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13500-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13500-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13500.01-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0
  * drivers: thermal: limits-dcvs: Always build driver
  * drivers: power: supply: Disable battery capacity learning
  * defconfig: revert back to 100Hz tick
  * Merge tag 'LA.UM.9.12.r1-13400.02-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0
  * build: version 2
  * build: its Snow time !!!
  * drivers: usb: dwc3: Specify sync probe for dwc3-of-simple.
  * drivers: usb: dwc3: Specify sync probe for msm-dwc3 driver
  * drivers: usb: dwc3: Specify sync probe for dwc3 driver
  * drivers: usb: pd: remove fix pd output for 5v
  * Revert "usb: pd: Register typec partner in case AAA is connected"
  * PM / sleep: Skip OOM killer toggles when kernel is compiled for Android
  * drm: msm: optimize interpolation
  * techpack: drm: sde: fix a race condition
  * platform: msm: Fix dangerous relocation
  * defconfig: Bump SLMK minfree & timeout
  * power: process: Use lesser time to enter sleep
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * qos: Don't allow userspace to impose restrictions on CPU idle levels
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * dts: kona-gpu: Remove qos active latency node
  * treewide: use mi drm notifier
  * drivers: power: add timeouts to wakelocks
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * ARM64: umi: overlay gpu data from kona-v2-gpu.dtsi
  * arm64: dts: kona-v2-gpu: Overclock to 670mhz
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * gitignore: add out directory
  * add yarpiins build stuff
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/defconfig: apollo: Bringup custom defconfig
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * Revert "scsi: ufs: increase query timeout"
  * Revert "drivers: Add support for memory fragmentation test simulator driver"
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * ANDROID: sched: Exempt paused CPU from nohz idle balance
  * ARM64: configs: Make configs required to pass CTS tests mandatory
  * ARM64/defconfig: umi: Disable LSE atomics
  * ARM64/defconfig: umi: Use a timer frequency of 100 Hz
  * arm64: Inline the spin lock function family
  * ARM64/defconfig: umi: Disable unused errata
  * ARM64/defconfig: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * UPSTREAM: arm64: link with -z norelro for LLD or aarch64-elf
  * techpack: display: msm: dsi: set doze brightness for aod correctly
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64/defconfig: umi: Disable some unuse drivers
  * ARM64/defconfig: umi: Disable stability debug configs Disable the following stability debug configs for shipping: - CONFIG_EDAC_KRYO_ARM64_PANIC_ON_UE
  * ARM64/defconfig: umi: Enable LLD, RELR, Clang ThinLTO optimizations
  * ANDROID: sched: EAS: take cstate into account when selecting idle core
  * ARM64/defconfig: umi: Enable jump label
  * arm64/kernel: jump_label: Switch to relative references
  * locking/static_key: Add support for deferred static branches
  * jump_label: Fix NULL dereference bug in __jump_label_mod_update()
  * jump_label: Annotate entries that operate on __init code earlier
  * jump_label: Implement generic support for relative references
  * jump_label: Abstract jump_entry member accessors
  * jump_label: Use static_key_linked() accessor
  * mm/vmalloc.c: switch to WARN_ON() and move it under unlink_va()
  * mm/vmalloc.c: get rid of one single unlink_va() when merge
  * mm/vmalloc.c: preload a CPU with one object for split purpose
  * mm/vmalloc.c: remove "node" argument
  * mm/vmalloc.c: fix typo in comment
  * vmalloc: export __vmalloc_node_range for CONFIG_TEST_VMALLOC_MODULE
  * mm/vmalloc: pass VM_USERMAP flags directly to __vmalloc_node_range()
  * mm/vmalloc: do not call kmemleak_free() on not yet accounted memory
  * mm/vmalloc.c: make vmalloc_32_user() align base kernel virtual address to SHMLBA
  * mm: convert totalram_pages and totalhigh_pages variables to atomic
  * vfree: add debug might_sleep()
  * mm/vmalloc.c: improve vfree() kerneldoc
  * drivers: scsi: ufs: Remove unused d_lu_wb_buf_alloc
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64/defconfig: umi: Disable QHEE kernel memory protection
  * qcacld-3.0: Free a bunch of pkts at once
  * Makefile: Use O3 optimization level for Clang LTO
  * Makefile: Use -O3 optimization level
  * drivers: thermal: Don't qualify thermal polling as high priority
  * ARM64/defconfig: umi: Enable power efficient workqueues
  * ARM64/defconfig: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64/defconfig: Disable serial console support
  * arm64: Don't build legacy QCOM DTS.
  * rcu: fix a performance regression
  * qcacmn: Fix build error with !IPA_OFFLOAD
  * ARM64/defconfig: umi: Disable qti core control and sched autogroup
  * ARM: dts: kona: Disable IRQ debugging
  * ARM64/defconfig: umi: Disable some debug drivers
  * f2fs: Demote GC thread to idle scheduler class
  * f2fs: Set ioprio of GC kthread to idle
  * f2fs: Enlarge min_fsync_blocks to 20
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * UPSTREAM: zram: move backing_dev under macro CONFIG_ZRAM_WRITEBACK
  * UPSTREAM: zram: fix broken page writeback
  * UPSTREAM: zram: fix return value on writeback_store
  * UPSTREAM: zram: support page writeback
  * BACKPORT: zcomp: Use ARRAY_SIZE() for backends list
  * BACKPORT: zram: Allocate struct zcomp_strm as per-CPU memory
  * Revert "zram: introduce zram_entry to prepare dedup functionality"
  * Revert "zram: implement deduplication in zram"
  * Revert "zram: make deduplication feature optional"
  * Revert "zram: compare all the entries with same checksum for deduplication"
  * Revert "zram: fix race condition while returning zram_entry refcount"
  * ASoC: pcm: Add 24bit playback audio support
  * ARM64: configs: enable CONFIG_WIREGUARD
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * arm64/defconfig: umi: Enable crypto LZ4
  * arm64/defconfig: umi: Enable zram-writeback support
  * ARM64: umi/defconfigs: Bringup custom config
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
  * block: zram: Fix idle/writeback string compare
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * drivers: gpu: msm: Only build adreno 6xx part
  * video: backlight: disable modules enabled by default
  * media: gspca: disable gspca module
  * net: disable bridge netfilter module
  * net: ipv4: disable modules which are enabled by default
  * qcacmn: Fix suspicious string concatenation warning in fwlog
  * rmnet_shs: Fix CFI violation in packet assignment
  * rmnet_perf: Fix CFI violation in packet deaggregation
  * ntfs: Fix ntfs_test_inode and ntfs_init_locked_inode function type If the kernel is built with CFI we hit a __cfi_check_fail while mounting a partition
  * power: supply: Classify Battery Monitor Systems as batteries
  * cpufreq: stats: Replace the global lock with atomic.
  * techpack: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * techpack: camera: Unmap secure buffers in secure usecase
  * subsystem_restart: Always performs soft resets when subsystems crash
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * Revert "dfc: Use alarm timer to trigger powersave work"
  * platform: msm: gsi: Export symbols only if compiled as module
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * scsi: ufs: Force enable write booster feature on UFS 3.1
  * scsi: ufs: Disable write booster feature support on UFS 2.2
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * treewide: Remove all Android.mk files
  * techpack: audio: Silence some logspam
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: Remove xiaomi early fingerprint wakeup optimization
  * audio: swr-mstr-ctrl: Fix unbalanced IRQ condition in swrm_runtime_suspend()
  * ARM64: configs: Enable XFRM_MIGRATE
  * ARM64: configs: Remove some debugging related features
  * ARM64: configs: Enable ARM64 Crypto Extensions SM3/4 and CRC32
  * ARM64: configs: Enable NTFS Filesystem
  * ARM64: configs: Enable ExFAT Filesystem
  * qcom: tz_log: Add support to extract trustzone logs from procfs
  * msm-poweroff: Store restart reason in panic
  * printk: Increase kernel log buffer size
  * smb5: Fix SDP current override for USB phy speed
  * mm: Disable watermark boost feature for K4.19
  * ARM64: configs: Skip file system sync in suspend
  * ARM64: configs: Disable PASR on kona devices
  * ARM64: configs: Reduce ION pool fill mark to 48MB
  * ARM64: configs: Build seperate DTB/DTBOs
  * ARM64: configs: Concatenate dtbs inside kernel image
  * ARM64: configs: Build Compressed Kernel Image
  * ARM64: configs: Build Qualcomm Atheros CLD WLAN module
  * ARM64: configs: Enable CPU frequency transition statistics
  * ARM64: configs: thyme: Enable support for block device writeback throttling
  * ARM64: configs: lmi: Enable AW2015 RGB LED support
  * ARM64: configs: lmi: Enable camera motor drivers
  * ARM64: configs: cas: Enable thermal dimming
  * ARM64: configs: apollo,cas: Enable panic on oops
  * ARM64: configs: apollo: Enable GPIO testing mode driver
  * ARM64: configs: Increase FRAME_WARN size to 4096
  * ARM64: configs: Enable Dynamic printk support
  * ARM64: configs: Disable MSM 11AD chip support
  * ARM64: configs: Enable xiaomi touchfeature driver
  * ARM64: configs: Disable USB CDC ADM support
  * ARM64: configs: Disable APP Armor security
  * ARM64: configs: Enable Ultrasound proximity driver
  * ARM64: configs: Enable USB Remote NDIS support
  * ARM64: configs: Disable USB Serial converter support
  * ARM64: configs: Disable V4L2 Videobuf2 Core
  * ARM64: configs: Enable SPI connected IR LED support
  * ARM64: configs: Enable PM8008 regulator driver
  * ARM64: configs: Enable device specific power supply drivers
  * ARM64: configs: Enable high performance logchar driver
  * ARM64: configs: Enable fastboot oem uart-enable command support
  * ARM64: configs: Disable support for Virtual terminal
  * ARM64: configs: Enable device specific fingerprint drivers
  * ARM64: configs: Enable AWINIC AW8697 haptics driver
  * ARM64: configs: Disable QTI Haptics support
  * ARM64: configs: Enable device specific touchscreen drivers
  * ARM64: configs: Disable Open-channel SSD target support
  * ARM64: configs: Enable CNSS QCA6390 chipset support
  * ARM64: configs: Enable QTI Smart Link Aggregation driver
  * ARM64: configs: Enable NAT, PEDIT and CSUM classifier Actions
  * ARM64: configs: Disable DEBUG_FS
  * ARM64: configs: Build Wilocity wil6210 driver
  * ARM64: configs: Enable Memory Technology Device (MTD) support
  * ARM64: configs: Disable Modules signature
  * ARM64: configs: Enable pstore support
  * ARM64: configs: Increase Kernel log buffer size
  * ARM64: configs: Enable Audit support
  * ARM64: configs: Enable device specific board platform
  * ARM64: configs: Generate xiaomi sm8250 devices configs from kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * bpf: Get the comm hash of the socket process stored inside sk buffer
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
  * fs: fuse: Implement FUSE passthrough
  * disp: msm: sde: Force SDE fd to start from 1
  * firmware: Import xiaomi touchscreens firmware
  * video: backlight: qcom-spmi-wled: Disable CABC on low brightness
  * video: backlight: Add support for thermal backlight dimming on J1S
  * video: backlight: Create brightness clone sysfs node
  * video: backlight: Notify on brightness node change
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * staging: android: ion: adjust system heap pool orders
  * staging: android: ion: Add xiaomi ion camera heap modifications
  * spi: gen-qcom: increase transfer timeout
  * soc: qcom: Add socinfo support for xiaomi sm8250 and sm7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
  * scsi: ufs: increase query timeout
  * mtd: devices: block2mtd: Sync scsi I/O on panic
  * debug: Add support to dump kmsg logs in kernel panic/oops using mtdoops
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * pinctrl: msm: Disable access to specified reserved gpios
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * cnss2: Add support for loading different bdwlan firmwares
  * net: cnss2: Add paramter to disable NV MAC support
  * net: cnss2: Increase cnss-daemon qmi timeout
  * media: rc: Add xiaomi modifications to IR driver
  * media: msm: cvp: Replace fence request handler kthread with workqueue
  * drivers: leds: Allow switching between dual flashlight leds
  * drivers: leds: Add support for AWINIC AW2015 3 Channel LED driver
  * drivers: cpuidle: Add xiaomi parameter to disable cpuidle and lpm
  * drivers: iio: Add support for Xiaomi ultrasound proximity sensor
  * drm: msm: Import xiaomi drm modifications
  * thermal: bcl_pmic5: Apply xiaomi thermal modifications
  * cpufreq: qcom: Always report maximum value of freq in limits_mitigation_notify()
  * drivers: thermal: Switch to CPUFREQ_ADJUST
  * msm: adsprpc: Handle UAF in process shell memory
  * mhi: mhi_qcom: Apply xiaomi modifications to sdx55 modem
  * drivers: misc: Import xiaomi GPIO testing mode driver
  * drivers: misc: Import Texas Instruments H-Bridge Stepper Motor Driver
  * drivers: misc: Import AKM Sensortecs AK09970 HALL sensor driver
  * block: Disable preemption before request_fn calls
  * ARM64: dts: qcom: Import kona mtp devicetree
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * input: touchscreen: Add support for xiaomi touchfeature dirver
  * input: touchscreen: Add support for Novatek NT36672c SPI touchscreen driver
  * input: touchscreen: Add support for GOODIX GT9886 touchscreen driver
  * input: touchscreen: Add support for STM FTS521 touchscreen driver
  * input: touchscreen: Add support for FocalTech ft8719/ft5452 touchscreen drivers
  * input: touchscreen: focaltech_spi: Don't ignore firmware files
  * input: touchscreen: Add support for FocalTech ft3658 touchscreen driver
  * input: touchscreen: Disable caf default touchscreen drivers
  * input: misc: aw8697_haptic: Add support to reload firmware for umi
  * input: misc: Add support for AWINIC AW8696 Haptic driver
  * input: fingerprint: Add support for GOODIX FOD fingerprint driver
  * input: fingerprint: Add support for GOODIX fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * drivers: Add support for memory fragmentation test simulator driver
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * techpack: audio: nuke xlogchar
  * Merge commit '99646a7c5d9584934073bd5c49438993f44c2f31' into android11-base
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * Add 'arch/arm64/boot/dts/vendor/' from commit 'bd2d6b0afa1f8aba19e41ce3bc29c16745595efa'
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * techpack: data: Build high performance ipa/rmnet drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * Revert "selinux: Relocate ss_initialized and selinux_enforcing to separate 4k"
  * ipa3: fix improper size checks
  * msm: ipa: Fix Makefile
  * kbuild: Remove gcc-wrapper
  * add toggle for disabling newly added USB devices
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * gitignore: dont ignore dts and techpack
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * techpack/pinctrl-lpi: initialise at late_initcall
  * techpack: audio: makefile: do not export all the variables
  * techpack: audio: Correct symlinks
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into lineage-19.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into lineage-19.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/camera-kernel into lineage-19.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into lineage-19.0
  * Merge 82146398a44b8081ef002068af683b1f44f4b226 on remote branch
  * Merge "ipa: Null persistent pointers after free"
  * Merge "leds: qpnp-flash-v2: Add support for dynamic torch current update"
  * Merge "qdss_bridge: handle usb write done event"
  * ipa: Null persistent pointers after free
  * qdss_bridge: handle usb write done event
  * leds: qpnp-flash-v2: Add support for dynamic torch current update
  * thermal: Update copyright info for ADC_TM driver
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge "diag: Update log and event mask code ranges"
  * Merge "defconfig: Enable SPMI TEMP ALARM driver for SPF targets"
  * Merge "drivers: thermal: Update a variable type in QMI encode logic"
  * defconfig: Enable SPMI TEMP ALARM driver for SPF targets
  * Merge "msm: kgsl: Update the IFPC power up reglist"
  * msm: ipa3: Fix to NULL terminate the header pointer in proc header table
  * Merge 4bd53b8750ce1a2bb7e1e9567beba529e85503b8 on remote branch
  * Merge "media: uvcvideo: Use cached memory for USB transfers"
  * Merge "usb: dwc3: Prevent deadlock when wakeup happens through pwr event irq"
  * usb: dwc3: Prevent deadlock when wakeup happens through pwr event irq
  * Merge "thermal: Read raw values for ADC_TM calibration channels"
  * Merge "qdss_bridge: fix stuck issue when driver remove"
  * thermal: Read raw values for ADC_TM calibration channels
  * Merge "BACKPORT: dma-buf: Move dma_buf_release() from fops to dentry_ops"
  * Merge "defconfig: Enable Novatek NT36xxx Touch for tron target"
  * Merge "power: smb1398-charger: Toggle UVLO for Slave CP on USB removal"
  * qdss_bridge: fix stuck issue when driver remove
  * media: uvcvideo: Use cached memory for USB transfers
  * defconfig: Enable Novatek NT36xxx Touch for tron target
  * Merge "cnss2: Dump PCIE SOC scratch registers along with mhi reg dumps"
  * Merge "ANDROID: mm: use raw seqcount variants in vm_write_*"
  * Merge "defconfig: Enable PMIC related configs for MSM8937_32"
  * cnss2: Dump PCIE SOC scratch registers along with mhi reg dumps
  * Merge "net: qrtr: Change error logging in callback"
  * ANDROID: mm: use raw seqcount variants in vm_write_*
  * Merge "defconfig: Enable Charger/LED/RTC configs for MSM8937_32go"
  * Merge "power: linear-charger: Report the correct battery status at SOC=0"
  * msm: kgsl: Update the IFPC power up reglist
  * Merge "msm: ipa3: Add wait queue for the adpl"
  * Merge "defconfig: arm64: msm: 8937-Turn on coresight configs"
  * Merge "cnss2: Update bound checks for sbl reg dumps to SRAM mem range"
  * Merge "soc: qcom: memory_dump: Support ETB/ETR register dump"
  * Merge "defconfig: arm: msm: 8937-Turn on coresight configs"
  * Merge "msm: ipa3: Fix to unlock mutex before return"
  * msm: ipa3: Add wait queue for the adpl
  * msm: ipa3: Fix to unlock mutex before return
  * cnss2: Update bound checks for sbl reg dumps to SRAM mem range
  * cnss2: Fix the pbl log sram start address for QCA6490
  * defconfig: arm: msm: 8937-Turn on coresight configs
  * arm: msm: Add support for coresight etr for 32 bit
  * soc: qcom: memory_dump: Support ETB/ETR register dump
  * defconfig: arm64: msm: 8937-Turn on coresight configs
  * Merge "msm: npu: handle system shutdown/reboot event properly"
  * Merge 8eaf52d7f00ce80a35524f68b0339ce9a8ed55c2 on remote branch
  * Merge "net: qrtr: fix a kernel-infoleak in qrtr_recvmsg()"
  * Merge "tty: Fix ->session locking"
  * drivers: thermal: Update a variable type in QMI encode logic
  * BACKPORT: dma-buf: Move dma_buf_release() from fops to dentry_ops
  * power: smb1398-charger: Toggle UVLO for Slave CP on USB removal
  * msm: npu: handle system shutdown/reboot event properly
  * Merge "USB: gadget: f_uvc: Enable more controls for CT and PU"
  * dma-mapping-fast: Fix iova address leak with non-zero scatterlist offset
  * net: qrtr: Change error logging in callback
  * Merge "net: qualcomm: rmnet: enable support for cksumv3"
  * Merge "usb: dwc3: dwc3-msm: optimize perf vote work"
  * defconfig: Enable PMIC related configs for MSM8937_32
  * defconfig: Enable PMIC related configs for MSM8937_64
  * defconfig: Enable Charger/LED/RTC configs for MSM8937_32go
  * power: linear-charger: Report the correct battery status at SOC=0
  * power: charger/fg: Add charger/fg/bms drivers for QM215
  * Merge "defconfig: Enable ADC thermal and SPMI configs for SPF targets"
  * defconfig: Enable ADC thermal and SPMI configs for SPF targets
  * thermal: Modify qpnp_adc_tm driver for IIO framework
  * Merge "msm: ipa3: Queue nop desc again if it fails"
  * thermal: qpnp-adc: Add snapshot of qpnp-adc-tm driver
  * Merge "msm: kgsl: Add timeline traces"
  * msm: ipa3: Queue nop desc again if it fails
  * diag: Drop packets to avoid memory exhaustion
  * Merge "msm-ipa: fix use-after-free of rt_tbl"
  * Merge "serial: msm_geni_serial: Avoid port pointer access for earlyconsole case"
  * msm-ipa: fix use-after-free of rt_tbl
  * net: qualcomm: rmnet: enable support for cksumv3
  * Merge "rpmsg: glink: Change error logging to avoid throttling"
  * serial: msm_geni_serial: Avoid port pointer access for earlyconsole case
  * rpmsg: glink: Change error logging to avoid throttling
  * USB: gadget: f_uvc: Enable more controls for CT and PU
  * Merge "USB: gadget: f_uvc: Enable required controls for CT and PU"
  * Merge "wait: add wake_up_sync()"
  * Merge "fuse: give wakeup hints to the scheduler"
  * msm: kgsl: Add timeline traces
  * msm: kgsl: Add software timelines
  * Merge "serial: msm_geni_serial: Reduce stale delay added in stop_rx_sequencer"
  * serial: msm_geni_serial: Reduce stale delay added in stop_rx_sequencer
  * Merge "usb: f_mtp: Limit MTP Tx req length to 16k for ChipIdea"
  * msm: kgsl: Fix out of bound write in adreno_profile_submit_time
  * Merge "msm: ipa: Fix to free up all pending EOB pages"
  * usb: f_mtp: Limit MTP Tx req length to 16k for ChipIdea
  * fuse: give wakeup hints to the scheduler
  * wait: add wake_up_sync()
  * msm: ipa: Fix to free up all pending EOB pages
  * msm: kgsl: Add the user API definition for timelines
  * Merge 01bf131abc089258714defda48b625be8de41916 on remote branch
  * Add support for new HSP version
  * Merge "msm: ipa3: increasing the uC interrupt timeout value"
  * Merge "msm: synx: acquire ref of synx handle in each function"
  * usb: dwc3: dwc3-msm: optimize perf vote work
  * i2c: i2c-qcom-geni: Change the high and low time for SCL
  * net: qrtr: fix a kernel-infoleak in qrtr_recvmsg()
  * tty: Fix ->session locking
  * msm: ipa3: increasing the uC interrupt timeout value
  * Merge "soc: qcom: Set QOS only to silver cluster"
  * Merge "msm: synx: fix synx_release_core race condition"
  * msm: kgsl: Fix out of bound write in adreno_profile_submit_time
  * diag: Update log and event mask code ranges
  * Merge "soc: qcom: smem: Update size of legacy partition"
  * soc: qcom: Set QOS only to silver cluster
  * Merge "dwc3-msm: Move override usb speed functionality outside edev check"
  * Merge "usb: f_mtp: Don't handle OS descriptors from MTP driver"
  * dwc3-msm: Move override usb speed functionality outside edev check
  * Merge "cnss2: Update board data file name for certain RF chip ID"
  * Merge "block: pm: Fix possible unbalanced nr_pending"
  * cnss2: Update board data file name for certain RF chip ID
  * Merge "regulator: qcom_pm8008-regulator: add support for PMIC PM8010"
  * platform: qpnp-revid: Add REVID support for PM8010
  * regulator: qcom_pm8008-regulator: add support for PMIC PM8010
  * regulator: qcom_pm8008: allow multiple PM8008 instances with unique names
  * Merge "defconfig: kona: Add CONFIG_AQFWD for RB5 board"
  * block: pm: Fix possible unbalanced nr_pending
  * Merge "power: supply: qpnp-smb5: specify different ICL for QC2 9V/12V level"
  * Merge "rpmsg: glink: do not break from interrupt handler"
  * Merge 93d96157e9245f165d755dd77b85b126f08fae85 on remote branch
  * usb: f_mtp: Don't handle OS descriptors from MTP driver
  * power: supply: qpnp-smb5: specify different ICL for QC2 9V/12V level
  * Merge "phy: msm: usb: Check VBUS state before accessing registers in suspend"
  * Merge "msm: adsprpc: Allocate buffer taking NULL byte into consideration"
  * phy: msm: usb: Check VBUS state before accessing registers in suspend
  * phy: msm: usb: Fail suspend if psy_type is USB or USB_CDP
  * USB: phy: msm: Apply DP pulse for CDP during bootup from sm_work
  * Merge "soc: qcom: secure_buffer: Fix the parameter passing to dmac_flush_range"
  * msm: adsprpc: Allocate buffer taking NULL byte into consideration
  * Merge "uapi: Add UAPI headers for slatecom_interface driver"
  * Merge "f2fs: don't sleep while grabing nat_tree_lock"
  * Merge "fbdev: msm: Avoid runtime sus/res for DP/HDMI"
  * fbdev: msm: Avoid runtime sus/res for DP/HDMI
  * Merge "Revert "qseecom: Add shmbrdige to allocate memory""
  * Merge "soc: qcom: smsm: Add wakeup capable flags to SMSM IRQ"
  * uapi: Add UAPI headers for slatecom_interface driver
  * Merge "ANDROID: xt_qtaguid: fix UAF race"
  * soc: qcom: smsm: Add wakeup capable flags to SMSM IRQ
  * f2fs: don't sleep while grabing nat_tree_lock
  * Ignore -106 error while opening channel
  * Merge "arm64: defconfig: Set qrtr wakeup for lito"
  * Merge "byte-cntr: Don't write csr register when byte-cntr is disabled"
  * byte-cntr: Don't write csr register when byte-cntr is disabled
  * arm64: defconfig: Set qrtr wakeup for lito
  * net: qrtr: Make wakeup timeout configurable
  * Merge "mhi: core: Avoid race condition mhi channel prepare and M0 event"
  * mhi: core: Avoid race condition mhi channel prepare and M0 event
  * Merge "msm: ipa3: Fix to handle zero length frag skb packet"
  * msm: synx: acquire ref of synx handle in each function
  * msm: ipa3: Fix to handle zero length frag skb packet
  * Merge "cnss_utils: Update the copyright years of file to 2017, 2019-2021"
  * cnss_utils: Update the copyright years of file to 2017, 2019-2021
  * Merge "clk: qcom: debugcc: Remove the gcc_camss_camnoc clocks"
  * Merge "FM: mutex changes modified"
  * FM: mutex changes modified
  * Merge "Revert "ANDROID: net: ipv4: sysfs_net_ipv4: Add sysfs-based knobs for controlling TCP window size""
  * Merge "coresight: byte-cnter: limit error log output"
  * Merge "coresight: byte-cnter: Add ETR status check in bypass notifier"
  * Revert "qseecom: Add shmbrdige to allocate memory"
  * Merge "serial: msm_geni_serial: Enable SW flow on in runtime suspend"
  * Merge "coresight: byte-cntr: Add ETR status check in bypass notifier"
  * Revert "ANDROID: net: ipv4: sysfs_net_ipv4: Add sysfs-based knobs for controlling TCP window size"
  * Revert "RFC: ANDROID: net: ipv4: sysfs_net_ipv4: Fix TCP window size controlling knobs"
  * ANDROID: xt_qtaguid: fix UAF race
  * coresight: byte-cntr: Add ETR status check in bypass notifier
  * clk: qcom: debugcc: Remove the gcc_camss_camnoc clocks
  * usb: f_fs: Avoid invalid pointer access in ffs_fs_get_tree
  * serial: msm_geni_serial: Enable SW flow on in runtime suspend
  * Merge "defconfig: msm: Enable RPM SMD cooling device driver for KHAJE"
  * Merge "irqchip: mpm: Add mpm mapping for Khaje"
  * defconfig: msm: Enable RPM SMD cooling device driver for KHAJE
  * Merge "usb: pd: Clear vdm_tx if SVDM message is sent on SOP'"
  * Merge "cnss2: Assert on FW memory allocation failure"
  * coresight: byte-cnter: limit error log output
  * coresight: byte-cnter: Add ETR status check in bypass notifier
  * Merge "f2fs: change to use rwsem for cp_mutex"
  * msm: synx: fix synx_release_core race condition
  * irqchip: mpm: Add mpm mapping for Khaje
  * Merge "dma-mapping-fast: Fix sg-list length calculation in fast_smmu_unmap_sg()"
  * f2fs: change to use rwsem for cp_mutex
  * cnss2: Assert on FW memory allocation failure
  * Merge "msm: kgsl: Keep the context alive until its fences signal"
  * msm: kgsl: Keep the context alive until its fences signal
  * soc: qcom: secure_buffer: Fix the parameter passing to dmac_flush_range
  * Merge "cnss_utils: Increase unsafe channel max num for 6G"
  * cnss_utils: Increase unsafe channel max num for 6G
  * Merge "driver: Fix compilation error with new sdclang 12"
  * Merge "clk: qcom: gpucc: Update voltage fmax table for GPU clock for KHAJE"
  * Merge "defconfig: Enable CONFIG_ION_SYSTEM_HEAP for bengal"
  * clk: qcom: gpucc: Update voltage fmax table for GPU clock for KHAJE
  * Merge "usb: phy: qusb: Add wrapper function for phy reset"
  * Merge "msm: kgsl: Use worker to put refcount on mem entry"
  * Merge "drivers: thermal: virtual-sensor: Add new virtual sensor for MSM8917"
  * Merge "qseecom: Add shmbrdige to allocate memory"
  * Merge "futex: Handle faults correctly for PI futexes"
  * Merge "msm: ipa: update the iommu mapping for WDI rings"
  * Merge "msm: ipa3: Enable hardbyte limit for WAN consumer pipe"
  * msm: ipa3: Enable hardbyte limit for WAN consumer pipe
  * dma-mapping-fast: Fix sg-list length calculation in fast_smmu_unmap_sg()
  * qseecom: Add shmbrdige to allocate memory
  * msm: kgsl: Use worker to put refcount on mem entry
  * usb: phy: qusb: Add wrapper function for phy reset
  * usb: pd: Clear vdm_tx if SVDM message is sent on SOP'
  * drivers: thermal: virtual-sensor: Add new virtual sensor for MSM8917
  * defconfig: Enable CONFIG_ION_SYSTEM_HEAP for bengal
  * futex: Handle faults correctly for PI futexes
  * futex: Simplify fixup_pi_state_owner()
  * soc: qcom: smem: Update size of legacy partition
  * driver: Fix compilation error with new sdclang 12
  * defconfig: kona: Add CONFIG_AQFWD for RB5 board
  * USB: gadget: f_uvc: Enable required controls for CT and PU
  * rpmsg: glink: do not break from interrupt handler
  * msm: ipa: update the iommu mapping for WDI rings

n.n.n / 2021-11-30
==================

  * x
  * Revert "sched: Import sched_irq_work_queue()"
  * Revert "kernel: sched: remove capabilities of show_state_filter_single"
  * Revert "Revert "UPSTREAM: sched: idle: Avoid retaining the tick when it has been stopped""
  * Revert "sched/rt: Fix RT utilization tracking during policy change"
  * Revert "sched/rt: Fix Deadline utilization tracking during policy change"
  * Revert "sched/{rt,deadline}: Fix set_next_task vs pick_next_task"
  * Revert "sched/fair: Use non-atomic cpumask_{set,clear}_cpu()"
  * Revert "sched/wait: Deduplicate code with do-while"
  * Revert "sched/core: Streamle calls to task_rq_unlock()"
  * Revert "sched/topology: Don't set SD_BALANCE_WAKE on cpuset domain relax"
  * Revert "sched/topology: Don't try to build empty sched domains"
  * Revert "sched/core: Use SCHED_RR in place of SCHED_FIFO for all users"
  * Revert "sched: fair: avoid little cpus due to sync, prev bias"
  * Revert "cpufreq: schedutil: fix check for stale utilization values"
  * Revert "sched: Do not reduce perceived CPU capacity while idle"
  * Revert "cpufreq: schedutil: Smoothen WALT predicted load boosting"
  * Revert "sched: features: Disable EAS_PREFER_IDLE"
  * Revert "sched/features: Fix hrtick reprogramming"
  * build: its Snow time !!!
  * drivers: usb: dwc3: Specify sync probe for dwc3-of-simple.
  * drivers: usb: dwc3: Specify sync probe for msm-dwc3 driver
  * drivers: usb: dwc3: Specify sync probe for dwc3 driver
  * drivers: usb: pd: remove fix pd output for 5v
  * Revert "usb: pd: Register typec partner in case AAA is connected"
  * PM / sleep: Skip OOM killer toggles when kernel is compiled for Android
  * drm: msm: optimize interpolation
  * techpack: drm: sde: fix a race condition
  * platform: msm: Fix dangerous relocation
  * defconfig: Bump SLMK minfree & timeout
  * power: process: Use lesser time to enter sleep
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * qos: Don't allow userspace to impose restrictions on CPU idle levels
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * dts: kona-gpu: Remove qos active latency node
  * treewide: use mi drm notifier
  * drivers: power: add timeouts to wakelocks
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * ARM64: umi: overlay gpu data from kona-v2-gpu.dtsi
  * arm64: dts: kona-v2-gpu: Overclock to 670mhz
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * gitignore: add out directory
  * add yarpiins build stuff
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/defconfig: apollo: Bringup custom defconfig
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * Revert "scsi: ufs: increase query timeout"
  * Revert "drivers: Add support for memory fragmentation test simulator driver"
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * ANDROID: sched: Exempt paused CPU from nohz idle balance
  * ARM64: configs: Make configs required to pass CTS tests mandatory
  * ARM64/defconfig: umi: Disable LSE atomics
  * ARM64/defconfig: umi: Use a timer frequency of 100 Hz
  * arm64: Inline the spin lock function family
  * ARM64/defconfig: umi: Disable unused errata
  * sched/features: Fix hrtick reprogramming
  * sched: features: Disable EAS_PREFER_IDLE
  * cpufreq: schedutil: Smoothen WALT predicted load boosting
  * sched: Do not reduce perceived CPU capacity while idle
  * cpufreq: schedutil: fix check for stale utilization values
  * sched: fair: avoid little cpus due to sync, prev bias
  * sched/core: Use SCHED_RR in place of SCHED_FIFO for all users
  * sched/topology: Don't try to build empty sched domains
  * sched/topology: Don't set SD_BALANCE_WAKE on cpuset domain relax
  * sched/core: Streamle calls to task_rq_unlock()
  * sched/wait: Deduplicate code with do-while
  * sched/fair: Use non-atomic cpumask_{set,clear}_cpu()
  * sched/{rt,deadline}: Fix set_next_task vs pick_next_task
  * sched/rt: Fix Deadline utilization tracking during policy change
  * sched/rt: Fix RT utilization tracking during policy change
  * Revert "UPSTREAM: sched: idle: Avoid retaining the tick when it has been stopped"
  * kernel: sched: remove capabilities of show_state_filter_single
  * sched: Import sched_irq_work_queue()
  * ARM64/defconfig: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * UPSTREAM: arm64: link with -z norelro for LLD or aarch64-elf
  * techpack: display: msm: dsi: set doze brightness for aod correctly
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64/defconfig: umi: Disable some unuse drivers
  * ARM64/defconfig: umi: Disable stability debug configs Disable the following stability debug configs for shipping: - CONFIG_EDAC_KRYO_ARM64_PANIC_ON_UE
  * ARM64/defconfig: umi: Enable LLD, RELR, Clang ThinLTO optimizations
  * ANDROID: sched: EAS: take cstate into account when selecting idle core
  * ARM64/defconfig: umi: Enable jump label
  * arm64/kernel: jump_label: Switch to relative references
  * locking/static_key: Add support for deferred static branches
  * jump_label: Fix NULL dereference bug in __jump_label_mod_update()
  * jump_label: Annotate entries that operate on __init code earlier
  * jump_label: Implement generic support for relative references
  * jump_label: Abstract jump_entry member accessors
  * jump_label: Use static_key_linked() accessor
  * mm/vmalloc.c: switch to WARN_ON() and move it under unlink_va()
  * mm/vmalloc.c: get rid of one single unlink_va() when merge
  * mm/vmalloc.c: preload a CPU with one object for split purpose
  * mm/vmalloc.c: remove "node" argument
  * mm/vmalloc.c: fix typo in comment
  * vmalloc: export __vmalloc_node_range for CONFIG_TEST_VMALLOC_MODULE
  * mm/vmalloc: pass VM_USERMAP flags directly to __vmalloc_node_range()
  * mm/vmalloc: do not call kmemleak_free() on not yet accounted memory
  * mm/vmalloc.c: make vmalloc_32_user() align base kernel virtual address to SHMLBA
  * mm: convert totalram_pages and totalhigh_pages variables to atomic
  * vfree: add debug might_sleep()
  * mm/vmalloc.c: improve vfree() kerneldoc
  * drivers: scsi: ufs: Remove unused d_lu_wb_buf_alloc
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64/defconfig: umi: Disable QHEE kernel memory protection
  * qcacld-3.0: Free a bunch of pkts at once
  * Makefile: Use O3 optimization level for Clang LTO
  * Makefile: Use -O3 optimization level
  * drivers: thermal: Don't qualify thermal polling as high priority
  * ARM64/defconfig: umi: Enable power efficient workqueues
  * ARM64/defconfig: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64/defconfig: Disable serial console support
  * arm64: Don't build legacy QCOM DTS.
  * rcu: fix a performance regression
  * qcacmn: Fix build error with !IPA_OFFLOAD
  * ARM64/defconfig: umi: Disable qti core control and sched autogroup
  * ARM: dts: kona: Disable IRQ debugging
  * ARM64/defconfig: umi: Disable some debug drivers
  * f2fs: Demote GC thread to idle scheduler class
  * f2fs: Set ioprio of GC kthread to idle
  * f2fs: Enlarge min_fsync_blocks to 20
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * UPSTREAM: zram: move backing_dev under macro CONFIG_ZRAM_WRITEBACK
  * UPSTREAM: zram: fix broken page writeback
  * UPSTREAM: zram: fix return value on writeback_store
  * UPSTREAM: zram: support page writeback
  * BACKPORT: zcomp: Use ARRAY_SIZE() for backends list
  * BACKPORT: zram: Allocate struct zcomp_strm as per-CPU memory
  * Revert "zram: introduce zram_entry to prepare dedup functionality"
  * Revert "zram: implement deduplication in zram"
  * Revert "zram: make deduplication feature optional"
  * Revert "zram: compare all the entries with same checksum for deduplication"
  * Revert "zram: fix race condition while returning zram_entry refcount"
  * ASoC: pcm: Add 24bit playback audio support
  * ARM64: configs: enable CONFIG_WIREGUARD
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * arm64/defconfig: umi: Enable crypto LZ4
  * arm64/defconfig: umi: Enable zram-writeback support
  * ARM64: umi/defconfigs: Bringup custom config
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
  * block: zram: Fix idle/writeback string compare
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * drivers: gpu: msm: Only build adreno 6xx part
  * video: backlight: disable modules enabled by default
  * media: gspca: disable gspca module
  * net: disable bridge netfilter module
  * net: ipv4: disable modules which are enabled by default
  * qcacmn: Fix suspicious string concatenation warning in fwlog
  * rmnet_shs: Fix CFI violation in packet assignment
  * rmnet_perf: Fix CFI violation in packet deaggregation
  * ntfs: Fix ntfs_test_inode and ntfs_init_locked_inode function type If the kernel is built with CFI we hit a __cfi_check_fail while mounting a partition
  * power: supply: Classify Battery Monitor Systems as batteries
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
  * cpufreq: stats: Replace the global lock with atomic.
  * techpack: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * techpack: camera: Unmap secure buffers in secure usecase
  * subsystem_restart: Always performs soft resets when subsystems crash
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * Revert "dfc: Use alarm timer to trigger powersave work"
  * platform: msm: gsi: Export symbols only if compiled as module
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * scsi: ufs: Force enable write booster feature on UFS 3.1
  * scsi: ufs: Disable write booster feature support on UFS 2.2
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * treewide: Remove all Android.mk files
  * techpack: audio: Silence some logspam
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: Remove xiaomi early fingerprint wakeup optimization
  * audio: swr-mstr-ctrl: Fix unbalanced IRQ condition in swrm_runtime_suspend()
  * ARM64: configs: Enable XFRM_MIGRATE
  * ARM64: configs: Remove some debugging related features
  * ARM64: configs: Enable ARM64 Crypto Extensions SM3/4 and CRC32
  * ARM64: configs: Enable NTFS Filesystem
  * ARM64: configs: Enable ExFAT Filesystem
  * qcom: tz_log: Add support to extract trustzone logs from procfs
  * msm-poweroff: Store restart reason in panic
  * printk: Increase kernel log buffer size
  * smb5: Fix SDP current override for USB phy speed
  * mm: Disable watermark boost feature for K4.19
  * ARM64: configs: Skip file system sync in suspend
  * ARM64: configs: Disable PASR on kona devices
  * ARM64: configs: Reduce ION pool fill mark to 48MB
  * ARM64: configs: Build seperate DTB/DTBOs
  * ARM64: configs: Concatenate dtbs inside kernel image
  * ARM64: configs: Build Compressed Kernel Image
  * ARM64: configs: Build Qualcomm Atheros CLD WLAN module
  * ARM64: configs: Enable CPU frequency transition statistics
  * ARM64: configs: thyme: Enable support for block device writeback throttling
  * ARM64: configs: lmi: Enable AW2015 RGB LED support
  * ARM64: configs: lmi: Enable camera motor drivers
  * ARM64: configs: cas: Enable thermal dimming
  * ARM64: configs: apollo,cas: Enable panic on oops
  * ARM64: configs: apollo: Enable GPIO testing mode driver
  * ARM64: configs: Increase FRAME_WARN size to 4096
  * ARM64: configs: Enable Dynamic printk support
  * ARM64: configs: Disable MSM 11AD chip support
  * ARM64: configs: Enable xiaomi touchfeature driver
  * ARM64: configs: Disable USB CDC ADM support
  * ARM64: configs: Disable APP Armor security
  * ARM64: configs: Enable Ultrasound proximity driver
  * ARM64: configs: Enable USB Remote NDIS support
  * ARM64: configs: Disable USB Serial converter support
  * ARM64: configs: Disable V4L2 Videobuf2 Core
  * ARM64: configs: Enable SPI connected IR LED support
  * ARM64: configs: Enable PM8008 regulator driver
  * ARM64: configs: Enable device specific power supply drivers
  * ARM64: configs: Enable high performance logchar driver
  * ARM64: configs: Enable fastboot oem uart-enable command support
  * ARM64: configs: Disable support for Virtual terminal
  * ARM64: configs: Enable device specific fingerprint drivers
  * ARM64: configs: Enable AWINIC AW8697 haptics driver
  * ARM64: configs: Disable QTI Haptics support
  * ARM64: configs: Enable device specific touchscreen drivers
  * ARM64: configs: Disable Open-channel SSD target support
  * ARM64: configs: Enable CNSS QCA6390 chipset support
  * ARM64: configs: Enable QTI Smart Link Aggregation driver
  * ARM64: configs: Enable NAT, PEDIT and CSUM classifier Actions
  * ARM64: configs: Disable DEBUG_FS
  * ARM64: configs: Build Wilocity wil6210 driver
  * ARM64: configs: Enable Memory Technology Device (MTD) support
  * ARM64: configs: Disable Modules signature
  * ARM64: configs: Enable pstore support
  * ARM64: configs: Increase Kernel log buffer size
  * ARM64: configs: Enable Audit support
  * ARM64: configs: Enable device specific board platform
  * ARM64: configs: Generate xiaomi sm8250 devices configs from kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * bpf: Get the comm hash of the socket process stored inside sk buffer
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
  * fs: fuse: Implement FUSE passthrough
  * disp: msm: sde: Force SDE fd to start from 1
  * firmware: Import xiaomi touchscreens firmware
  * video: backlight: qcom-spmi-wled: Disable CABC on low brightness
  * video: backlight: Add support for thermal backlight dimming on J1S
  * video: backlight: Create brightness clone sysfs node
  * video: backlight: Notify on brightness node change
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * staging: android: ion: adjust system heap pool orders
  * staging: android: ion: Add xiaomi ion camera heap modifications
  * spi: gen-qcom: increase transfer timeout
  * soc: qcom: Add socinfo support for xiaomi sm8250 and sm7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
  * scsi: ufs: increase query timeout
  * mtd: devices: block2mtd: Sync scsi I/O on panic
  * debug: Add support to dump kmsg logs in kernel panic/oops using mtdoops
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * pinctrl: msm: Disable access to specified reserved gpios
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * cnss2: Add support for loading different bdwlan firmwares
  * net: cnss2: Add paramter to disable NV MAC support
  * net: cnss2: Increase cnss-daemon qmi timeout
  * media: rc: Add xiaomi modifications to IR driver
  * media: msm: cvp: Replace fence request handler kthread with workqueue
  * drivers: leds: Allow switching between dual flashlight leds
  * drivers: leds: Add support for AWINIC AW2015 3 Channel LED driver
  * drivers: cpuidle: Add xiaomi parameter to disable cpuidle and lpm
  * drivers: iio: Add support for Xiaomi ultrasound proximity sensor
  * drm: msm: Import xiaomi drm modifications
  * thermal: bcl_pmic5: Apply xiaomi thermal modifications
  * cpufreq: qcom: Always report maximum value of freq in limits_mitigation_notify()
  * thermal: core: Implement thermal config switch
  * drivers: char: Add support for high performance log driver
  * mhi: mhi_qcom: Apply xiaomi modifications to sdx55 modem
  * drivers: misc: Import xiaomi GPIO testing mode driver
  * drivers: misc: Import Texas Instruments H-Bridge Stepper Motor Driver
  * drivers: misc: Import AKM Sensortecs AK09970 HALL sensor driver
  * block: Disable preemption before request_fn calls
  * ARM64: dts: qcom: Import kona mtp devicetree
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * input: touchscreen: Add support for xiaomi touchfeature dirver
  * input: touchscreen: Add support for Novatek NT36672c SPI touchscreen driver
  * input: touchscreen: Add support for GOODIX GT9886 touchscreen driver
  * input: touchscreen: Add support for STM FTS521 touchscreen driver
  * input: touchscreen: Add support for FocalTech ft8719/ft5452 touchscreen drivers
  * input: touchscreen: focaltech_spi: Don't ignore firmware files
  * input: touchscreen: Add support for FocalTech ft3658 touchscreen driver
  * input: touchscreen: Disable caf default touchscreen drivers
  * input: misc: aw8697_haptic: Add support to reload firmware for umi
  * input: misc: Add support for AWINIC AW8696 Haptic driver
  * input: fingerprint: Add support for GOODIX FOD fingerprint driver
  * input: fingerprint: Add support for GOODIX fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * drivers: Add support for memory fragmentation test simulator driver
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * techpack: audio: nuke xlogchar
  * Merge commit '99646a7c5d9584934073bd5c49438993f44c2f31' into android11-base
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * Add 'arch/arm64/boot/dts/vendor/' from commit 'bd2d6b0afa1f8aba19e41ce3bc29c16745595efa'
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * techpack: data: Build high performance ipa/rmnet drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * Revert "selinux: Relocate ss_initialized and selinux_enforcing to separate 4k"
  * ipa3: fix improper size checks
  * msm: ipa: Fix Makefile
  * kbuild: Remove gcc-wrapper
  * add toggle for disabling newly added USB devices
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * gitignore: dont ignore dts and techpack
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * techpack/pinctrl-lpi: initialise at late_initcall
  * techpack: audio: makefile: do not export all the variables
  * techpack: audio: Correct symlinks
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into lineage-19.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into lineage-19.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/camera-kernel into lineage-19.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into lineage-19.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge 8eaf52d7f00ce80a35524f68b0339ce9a8ed55c2 on remote branch
  * Merge "USB: gadget: f_uvc: Enable required controls for CT and PU"
  * Merge "wait: add wake_up_sync()"
  * Merge "fuse: give wakeup hints to the scheduler"
  * Merge "serial: msm_geni_serial: Reduce stale delay added in stop_rx_sequencer"
  * serial: msm_geni_serial: Reduce stale delay added in stop_rx_sequencer
  * Merge "usb: f_mtp: Limit MTP Tx req length to 16k for ChipIdea"
  * Merge "msm: ipa: Fix to free up all pending EOB pages"
  * usb: f_mtp: Limit MTP Tx req length to 16k for ChipIdea
  * fuse: give wakeup hints to the scheduler
  * wait: add wake_up_sync()
  * msm: ipa: Fix to free up all pending EOB pages
  * Add support for new HSP version
  * Merge "msm: ipa3: increasing the uC interrupt timeout value"
  * Merge "msm: synx: acquire ref of synx handle in each function"
  * i2c: i2c-qcom-geni: Change the high and low time for SCL
  * msm: ipa3: increasing the uC interrupt timeout value
  * Merge "soc: qcom: Set QOS only to silver cluster"
  * Merge "msm: synx: fix synx_release_core race condition"
  * msm: kgsl: Fix out of bound write in adreno_profile_submit_time
  * Merge "soc: qcom: smem: Update size of legacy partition"
  * soc: qcom: Set QOS only to silver cluster
  * Merge "dwc3-msm: Move override usb speed functionality outside edev check"
  * dwc3-msm: Move override usb speed functionality outside edev check
  * msm: synx: acquire ref of synx handle in each function
  * msm: synx: fix synx_release_core race condition
  * soc: qcom: smem: Update size of legacy partition
  * USB: gadget: f_uvc: Enable required controls for CT and PU

n.n.n / 2021-11-29
==================

  * x
  * drivers: usb: dwc3: Specify sync probe for dwc3-of-simple.
  * drivers: usb: dwc3: Specify sync probe for msm-dwc3 driver
  * drivers: usb: dwc3: Specify sync probe for dwc3 driver
  * drivers: usb: pd: remove fix pd output for 5v
  * Revert "usb: pd: Register typec partner in case AAA is connected"
  * PM / sleep: Skip OOM killer toggles when kernel is compiled for Android
  * drm: msm: optimize interpolation
  * techpack: drm: sde: fix a race condition
  * platform: msm: Fix dangerous relocation
  * defconfig: Bump SLMK minfree & timeout
  * power: process: Use lesser time to enter sleep
  * lib/sort: Move swap, cmp and cmp_r function types for wider use
  * media: lib/sort.c: implement sort() variant taking context argument
  * lib/sort.c: fix kernel-doc notation warnings
  * lib/sort: avoid indirect calls to built-in swap
  * lib/sort: use more efficient bottom-up heapsort variant
  * lib/sort: make swap functions more generic
  * lib/string: optimized memset
  * lib/string: optimized memmove
  * lib/string: optimized memcpy
  * arm64: strcmp: align to 64B cache line
  * arm64: Use optimized memcmp
  * mm: kmemleak: Don't die when memory allocation fails
  * defconfig: regen
  * qos: Don't allow userspace to impose restrictions on CPU idle levels
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * dts: kona-gpu: Remove qos active latency node
  * treewide: use mi drm notifier
  * drivers: power: add timeouts to wakelocks
  * defconfig: regen
  * cpufreq: schedutilX: Introduce initial bringup
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * defconfig: enable Simple LMK
  * simple_lmk: Thaw victims upon killing them
  * simple_lmk: Make the reclaim thread freezable
  * simple_lmk: Be extra paranoid if tasks can have no pages
  * mm: Increment kswapd_waiters for throttled direct reclaimers
  * simple_lmk: Use atomic_t instead of spinlock
  * simple_lmk: Fix 7fabc7afd81908f5ab065a8bac5358cdeda22c24
  * simple_lmk: Try to improve UX
  * mm: vmpressure: Fix rampant inaccuracies caused by stale data usage
  * mm: vmpressure: Fix a race that would erroneously clear accumulated data
  * mm: vmpressure: Ignore costly-order allocations for direct reclaim too
  * simple_lmk: Update do_send_sig_info() group argument for 4.18+
  * simple_lmk: Optimize victim finder to eliminate hard-coded adj ranges
  * simple_lmk: Cacheline-align the victims array and mm_free_lock on SMP
  * simple_lmk: Pass a custom swap function to sort()
  * simple_lmk: Skip victim reduction when all victims need to be killed
  * simple_lmk: Use MIN_FREE_PAGES wherever pages_needed is used
  * simple_lmk: Don't block in simple_lmk_mm_freed() on mm_free_lock
  * mm: vmpressure: Don't export tunables to userspace
  * simple_lmk: Update Kconfig description for VM pressure change
  * simple_lmk: Add !PSI dependency
  * simple_lmk: Print a message when the timeout is reached
  * VFS: use synchronize_rcu_expedited() in namespace_unlock()
  * simple_lmk: Remove unnecessary clean-up when timeout is reached
  * simple_lmk: Hold an RCU read lock instead of the tasklist read lock
  * mm: Don't stop kswapd on a per-node basis when there are no waiters
  * simple_lmk: Consider all positive adjs when finding victims
  * mm: vmpressure: Ignore allocation orders above PAGE_ALLOC_COSTLY_ORDER
  * mm: Don't warn on page allocation failures for OOM-killed processes
  * mm: Adjust tsk_is_oom_victim() for Simple LMK
  * mm: vmpressure: Don't cache the window size
  * mm: vmpressure: Interpret zero scanned pages as 100% pressure
  * mm: vmpressure: Don't exclude any allocation types
  * simple_lmk: Update adj targeting for Android 10
  * simple_lmk: Use vmpressure notifier to trigger kills
  * mm: vmpressure: account allocstalls only on higher pressures
  * mm: vmpressure: scale pressure based on reclaim context
  * mm: Stop kswapd early when nothing's waiting for it to free pages
  * simple_lmk: Include swap memory usage in the size of victims
  * simple_lmk: Relax memory barriers and clean up some styling
  * simple_lmk: Place victims onto SCHED_RR
  * simple_lmk: Add a timeout to stop waiting for victims to die
  * simple_lmk: Ignore tasks that won't free memory
  * simple_lmk: Simplify tricks used to speed up the death process
  * simple_lmk: Report mm as freed as soon as exit_mmap() finishes
  * simple_lmk: Mark victim thread group with TIF_MEMDIE
  * simple_lmk: Disable OOM killer when Simple LMK is enabled
  * simple_lmk: Print a message when there are no processes to kill
  * simple_lmk: Remove compat cruft not specific to 4.14
  * simple_lmk: Update copyright to 2020
  * simple_lmk: Don't queue up new reclaim requests during reclaim
  * simple_lmk: Increase default minfree value
  * simple_lmk: Clean up some code style nitpicks
  * simple_lmk: Make reclaim deterministic
  * simple_lmk: Fix broken multicopy atomicity for victims_to_kill
  * simple_lmk: Use proper atomic_* operations where needed
  * simple_lmk: Remove kthread_should_stop() exit condition
  * simple_lmk: Fix pages_found calculation
  * simple_lmk: Introduce Simple Low Memory Killer for Android
  * defconfig: enable wakelock blocker
  * boeffla_wl_blocker: don't block wakelocks by default
  * boeffla_wl_blocker: update to wakelock blocker driver v1.1.0
  * boeffla_wl_blocker: update to wakelock blocker driver v1.0.1
  * boeffla_wl_blocker: add generic wakelock blocker driver v1.0.0
  * ARM64: dts: Allow Adreno 650 to nap
  * Kernel: Disable GFS for better UI performance
  * Kernel/sched: Reduce Latency [Pafcholini]
  * Kernel/sched: Reduce latency for better responsiveness
  * defconfig: enable zen i/o scheduler
  * zen-iosched:: fix elevator ops
  * zen-iosched: cast parameters to unsigned long
  * block: zen: fix build on 3.18 kernel
  * block: add zen IO scheduler
  * ARM64: umi: overlay gpu data from kona-v2-gpu.dtsi
  * arm64: dts: kona-v2-gpu: Overclock to 670mhz
  * msm: kgsl: Report correct GPU frequency in sysfs
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * drm-msm-dsi: add backlight min option
  * msm: kgsl: adreno: Properly set GPU timeout
  * dts: kona: Set GPU idle timeout to 64 ms
  * cpufreq: schedutil: make iowait boost optional
  * kernel: Boost all CPUs to the max when userspace launches an app
  * cpu_input_boost: Add support for max boost freqs lower than min freq
  * cpu_input_boost: add support for boost freqs lower than min freq
  * cpu_input_boost: Allow configuration of each cluster's minimum idle frequency
  * cpu_input_boost: Allow configuration of each cluster's minimum frequency
  * cpu_input_boost: Expose compile-time settings as module parameters
  * cpu_input_boost: Introduce driver for event-based CPU boosting
  * cpumask: Add cpumasks for big, LITTLE, and prime CPU clusters
  * kcal: pie: fix used pcc setup for sde path
  * drm: msm: kcal: make the userspace module param vars static and use short instead of integers
  * drm: msm: kcal: hue, saturation, value and contrast adjustments
  * drm: msm: kcal: export rgb to userspace and clean it up
  * kcal: sde_hw: color proc v4: add RGB bias
  * adrenoboost: stats is now a pointer
  * adrenoboost: disable by default
  * adrenoboost: finetuning algorithm - scale it a bit down
  * msm_adreno_tz: add adrenoboost parameter
  * msm_performance: Kill boosts
  * defconfig: unset cpu boost
  * msm: kgsl: Turn debugfs off
  * msm: kgsl: Relax CPU latency requirements to save power
  * kgsl: run kgsl-workqueue as a high prio wq
  * kgsl: Increase priority of RT thread
  * devfreq: Use high priority workqueue
  * workqueue: change permissions to allow root control of wq_power_efficient toggle
  * drivers/mmc/core: expose software CRC check option to userspace
  * arch: arm64: dts: Correct SLPI/ADSP/SPSS/CDSP memory regions on LMI/CMI/UMI
  * arch/Kconfig: disable Shadow Call Stack by default
  * security: set INIT_STACK_NONE as default
  * treewide: build modules inline
  * gitignore: add out directory
  * add yarpiins build stuff
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  * ARM64: dts: Disable Xiaomi FOD dimlayer impl
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64/defconfig: apollo: Bringup custom defconfig
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * Revert "scsi: ufs: increase query timeout"
  * Revert "drivers: Add support for memory fragmentation test simulator driver"
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * ANDROID: sched: Exempt paused CPU from nohz idle balance
  * ARM64: configs: Make configs required to pass CTS tests mandatory
  * ARM64/defconfig: umi: Disable LSE atomics
  * ARM64/defconfig: umi: Use a timer frequency of 100 Hz
  * arm64: Inline the spin lock function family
  * ARM64/defconfig: umi: Disable unused errata
  * sched/features: Fix hrtick reprogramming
  * sched: features: Disable EAS_PREFER_IDLE
  * cpufreq: schedutil: Smoothen WALT predicted load boosting
  * sched: Do not reduce perceived CPU capacity while idle
  * cpufreq: schedutil: fix check for stale utilization values
  * sched: fair: avoid little cpus due to sync, prev bias
  * sched/core: Use SCHED_RR in place of SCHED_FIFO for all users
  * sched/topology: Don't try to build empty sched domains
  * sched/topology: Don't set SD_BALANCE_WAKE on cpuset domain relax
  * sched/core: Streamle calls to task_rq_unlock()
  * sched/wait: Deduplicate code with do-while
  * sched/fair: Use non-atomic cpumask_{set,clear}_cpu()
  * sched/{rt,deadline}: Fix set_next_task vs pick_next_task
  * sched/rt: Fix Deadline utilization tracking during policy change
  * sched/rt: Fix RT utilization tracking during policy change
  * Revert "UPSTREAM: sched: idle: Avoid retaining the tick when it has been stopped"
  * kernel: sched: remove capabilities of show_state_filter_single
  * sched: Import sched_irq_work_queue()
  * ARM64/defconfig: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * UPSTREAM: arm64: link with -z norelro for LLD or aarch64-elf
  * techpack: display: msm: dsi: set doze brightness for aod correctly
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64/defconfig: umi: Disable some unuse drivers
  * ARM64/defconfig: umi: Disable stability debug configs Disable the following stability debug configs for shipping: - CONFIG_EDAC_KRYO_ARM64_PANIC_ON_UE
  * ARM64/defconfig: umi: Enable LLD, RELR, Clang ThinLTO optimizations
  * ANDROID: sched: EAS: take cstate into account when selecting idle core
  * ARM64/defconfig: umi: Enable jump label
  * arm64/kernel: jump_label: Switch to relative references
  * locking/static_key: Add support for deferred static branches
  * jump_label: Fix NULL dereference bug in __jump_label_mod_update()
  * jump_label: Annotate entries that operate on __init code earlier
  * jump_label: Implement generic support for relative references
  * jump_label: Abstract jump_entry member accessors
  * jump_label: Use static_key_linked() accessor
  * mm/vmalloc.c: switch to WARN_ON() and move it under unlink_va()
  * mm/vmalloc.c: get rid of one single unlink_va() when merge
  * mm/vmalloc.c: preload a CPU with one object for split purpose
  * mm/vmalloc.c: remove "node" argument
  * mm/vmalloc.c: fix typo in comment
  * vmalloc: export __vmalloc_node_range for CONFIG_TEST_VMALLOC_MODULE
  * mm/vmalloc: pass VM_USERMAP flags directly to __vmalloc_node_range()
  * mm/vmalloc: do not call kmemleak_free() on not yet accounted memory
  * mm/vmalloc.c: make vmalloc_32_user() align base kernel virtual address to SHMLBA
  * mm: convert totalram_pages and totalhigh_pages variables to atomic
  * vfree: add debug might_sleep()
  * mm/vmalloc.c: improve vfree() kerneldoc
  * drivers: scsi: ufs: Remove unused d_lu_wb_buf_alloc
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64/defconfig: umi: Disable QHEE kernel memory protection
  * qcacld-3.0: Free a bunch of pkts at once
  * Makefile: Use O3 optimization level for Clang LTO
  * Makefile: Use -O3 optimization level
  * drivers: thermal: Don't qualify thermal polling as high priority
  * ARM64/defconfig: umi: Enable power efficient workqueues
  * ARM64/defconfig: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64/defconfig: Disable serial console support
  * arm64: Don't build legacy QCOM DTS.
  * rcu: fix a performance regression
  * qcacmn: Fix build error with !IPA_OFFLOAD
  * ARM64/defconfig: umi: Disable qti core control and sched autogroup
  * ARM: dts: kona: Disable IRQ debugging
  * ARM64/defconfig: umi: Disable some debug drivers
  * f2fs: Demote GC thread to idle scheduler class
  * f2fs: Set ioprio of GC kthread to idle
  * f2fs: Enlarge min_fsync_blocks to 20
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * UPSTREAM: zram: move backing_dev under macro CONFIG_ZRAM_WRITEBACK
  * UPSTREAM: zram: fix broken page writeback
  * UPSTREAM: zram: fix return value on writeback_store
  * UPSTREAM: zram: support page writeback
  * BACKPORT: zcomp: Use ARRAY_SIZE() for backends list
  * BACKPORT: zram: Allocate struct zcomp_strm as per-CPU memory
  * Revert "zram: introduce zram_entry to prepare dedup functionality"
  * Revert "zram: implement deduplication in zram"
  * Revert "zram: make deduplication feature optional"
  * Revert "zram: compare all the entries with same checksum for deduplication"
  * Revert "zram: fix race condition while returning zram_entry refcount"
  * ASoC: pcm: Add 24bit playback audio support
  * ARM64: configs: enable CONFIG_WIREGUARD
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * arm64/defconfig: umi: Enable crypto LZ4
  * arm64/defconfig: umi: Enable zram-writeback support
  * ARM64: umi/defconfigs: Bringup custom config
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
  * block: zram: Fix idle/writeback string compare
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * drivers: gpu: msm: Only build adreno 6xx part
  * video: backlight: disable modules enabled by default
  * media: gspca: disable gspca module
  * net: disable bridge netfilter module
  * net: ipv4: disable modules which are enabled by default
  * qcacmn: Fix suspicious string concatenation warning in fwlog
  * rmnet_shs: Fix CFI violation in packet assignment
  * rmnet_perf: Fix CFI violation in packet deaggregation
  * ntfs: Fix ntfs_test_inode and ntfs_init_locked_inode function type If the kernel is built with CFI we hit a __cfi_check_fail while mounting a partition
  * power: supply: Classify Battery Monitor Systems as batteries
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
  * cpufreq: stats: Replace the global lock with atomic.
  * techpack: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * techpack: camera: Unmap secure buffers in secure usecase
  * subsystem_restart: Always performs soft resets when subsystems crash
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * Revert "dfc: Use alarm timer to trigger powersave work"
  * platform: msm: gsi: Export symbols only if compiled as module
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * scsi: ufs: Force enable write booster feature on UFS 3.1
  * scsi: ufs: Disable write booster feature support on UFS 2.2
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * treewide: Remove all Android.mk files
  * techpack: audio: Silence some logspam
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: Remove xiaomi early fingerprint wakeup optimization
  * audio: swr-mstr-ctrl: Fix unbalanced IRQ condition in swrm_runtime_suspend()
  * ARM64: configs: Enable XFRM_MIGRATE
  * ARM64: configs: Remove some debugging related features
  * ARM64: configs: Enable ARM64 Crypto Extensions SM3/4 and CRC32
  * ARM64: configs: Enable NTFS Filesystem
  * ARM64: configs: Enable ExFAT Filesystem
  * qcom: tz_log: Add support to extract trustzone logs from procfs
  * msm-poweroff: Store restart reason in panic
  * printk: Increase kernel log buffer size
  * smb5: Fix SDP current override for USB phy speed
  * mm: Disable watermark boost feature for K4.19
  * ARM64: configs: Skip file system sync in suspend
  * ARM64: configs: Disable PASR on kona devices
  * ARM64: configs: Reduce ION pool fill mark to 48MB
  * ARM64: configs: Build seperate DTB/DTBOs
  * ARM64: configs: Concatenate dtbs inside kernel image
  * ARM64: configs: Build Compressed Kernel Image
  * ARM64: configs: Build Qualcomm Atheros CLD WLAN module
  * ARM64: configs: Enable CPU frequency transition statistics
  * ARM64: configs: thyme: Enable support for block device writeback throttling
  * ARM64: configs: lmi: Enable AW2015 RGB LED support
  * ARM64: configs: lmi: Enable camera motor drivers
  * ARM64: configs: cas: Enable thermal dimming
  * ARM64: configs: apollo,cas: Enable panic on oops
  * ARM64: configs: apollo: Enable GPIO testing mode driver
  * ARM64: configs: Increase FRAME_WARN size to 4096
  * ARM64: configs: Enable Dynamic printk support
  * ARM64: configs: Disable MSM 11AD chip support
  * ARM64: configs: Enable xiaomi touchfeature driver
  * ARM64: configs: Disable USB CDC ADM support
  * ARM64: configs: Disable APP Armor security
  * ARM64: configs: Enable Ultrasound proximity driver
  * ARM64: configs: Enable USB Remote NDIS support
  * ARM64: configs: Disable USB Serial converter support
  * ARM64: configs: Disable V4L2 Videobuf2 Core
  * ARM64: configs: Enable SPI connected IR LED support
  * ARM64: configs: Enable PM8008 regulator driver
  * ARM64: configs: Enable device specific power supply drivers
  * ARM64: configs: Enable high performance logchar driver
  * ARM64: configs: Enable fastboot oem uart-enable command support
  * ARM64: configs: Disable support for Virtual terminal
  * ARM64: configs: Enable device specific fingerprint drivers
  * ARM64: configs: Enable AWINIC AW8697 haptics driver
  * ARM64: configs: Disable QTI Haptics support
  * ARM64: configs: Enable device specific touchscreen drivers
  * ARM64: configs: Disable Open-channel SSD target support
  * ARM64: configs: Enable CNSS QCA6390 chipset support
  * ARM64: configs: Enable QTI Smart Link Aggregation driver
  * ARM64: configs: Enable NAT, PEDIT and CSUM classifier Actions
  * ARM64: configs: Disable DEBUG_FS
  * ARM64: configs: Build Wilocity wil6210 driver
  * ARM64: configs: Enable Memory Technology Device (MTD) support
  * ARM64: configs: Disable Modules signature
  * ARM64: configs: Enable pstore support
  * ARM64: configs: Increase Kernel log buffer size
  * ARM64: configs: Enable Audit support
  * ARM64: configs: Enable device specific board platform
  * ARM64: configs: Generate xiaomi sm8250 devices configs from kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * bpf: Get the comm hash of the socket process stored inside sk buffer
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
  * fs: fuse: Implement FUSE passthrough
  * disp: msm: sde: Force SDE fd to start from 1
  * firmware: Import xiaomi touchscreens firmware
  * video: backlight: qcom-spmi-wled: Disable CABC on low brightness
  * video: backlight: Add support for thermal backlight dimming on J1S
  * video: backlight: Create brightness clone sysfs node
  * video: backlight: Notify on brightness node change
  * Revert "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * Revert "usb: pd: Add support to disable pps capability"
  * Revert "USB: pd: Restart host mode in high speed if no usb3 & dp concurrency"
  * Revert "USB: pd: Add support for enabling PD2.0 only as source"
  * usb: pd: Import xiaomi usbpd modifications
  * usb: f_mtp: Set MTP interface same as PTP interface
  * usb: f_gsi: Load wireless controller rndis for non MSOS devices
  * usb: f_gsi: Set rndis over ethernet for MSOS devices
  * usb: configs: Implement device attribute for MSOS vendor
  * usb: configfs: Add function to disable usb power supply rechecking
  * usb: gadget: composite: Set bcdUSB to 2.0 for not superspeed gadget
  * usb: gadget: composite: Disable LPM and BESL support
  * usb: dwc3: Disable USB LPM Feature
  * tty: serial: Support fastboot oem uart-enable command
  * staging: android: ion: adjust system heap pool orders
  * staging: android: ion: Add xiaomi ion camera heap modifications
  * spi: gen-qcom: increase transfer timeout
  * soc: qcom: Add socinfo support for xiaomi sm8250 and sm7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
  * scsi: ufs: increase query timeout
  * mtd: devices: block2mtd: Sync scsi I/O on panic
  * debug: Add support to dump kmsg logs in kernel panic/oops using mtdoops
  * power: qcom: cas: Add support for 60W PD charging
  * power: qcom: Import xiaomi power supply modifications
  * Revert "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * pinctrl: msm: Disable access to specified reserved gpios
  * nfc: nq-nfc: Add xiaomi modifications for nfc bringup
  * Revert "NFC: Add support for core init command"
  * cnss2: Add support for loading different bdwlan firmwares
  * net: cnss2: Add paramter to disable NV MAC support
  * net: cnss2: Increase cnss-daemon qmi timeout
  * media: rc: Add xiaomi modifications to IR driver
  * media: msm: cvp: Replace fence request handler kthread with workqueue
  * drivers: leds: Allow switching between dual flashlight leds
  * drivers: leds: Add support for AWINIC AW2015 3 Channel LED driver
  * drivers: cpuidle: Add xiaomi parameter to disable cpuidle and lpm
  * drivers: iio: Add support for Xiaomi ultrasound proximity sensor
  * drm: msm: Import xiaomi drm modifications
  * thermal: bcl_pmic5: Apply xiaomi thermal modifications
  * cpufreq: qcom: Always report maximum value of freq in limits_mitigation_notify()
  * thermal: core: Implement thermal config switch
  * drivers: char: Add support for high performance log driver
  * mhi: mhi_qcom: Apply xiaomi modifications to sdx55 modem
  * drivers: misc: Import xiaomi GPIO testing mode driver
  * drivers: misc: Import Texas Instruments H-Bridge Stepper Motor Driver
  * drivers: misc: Import AKM Sensortecs AK09970 HALL sensor driver
  * block: Disable preemption before request_fn calls
  * ARM64: dts: qcom: Import kona mtp devicetree
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * drivers: input: touchscreen: nt36672c: Implement double_tap node
  * drivers: input: touchscreen: fts_521: Implement double_tap node
  * drivers: input: touchscreen: focaltech_spi: Implement double_tap node
  * drivers: input: touchscreen: focaltech_touch: Implement double_tap node
  * drivers: input: touchscreen: Add an interface to expose TP features to userspace
  * input: touchscreen: Add support for xiaomi touchfeature dirver
  * input: touchscreen: Add support for Novatek NT36672c SPI touchscreen driver
  * input: touchscreen: Add support for GOODIX GT9886 touchscreen driver
  * input: touchscreen: Add support for STM FTS521 touchscreen driver
  * input: touchscreen: Add support for FocalTech ft8719/ft5452 touchscreen drivers
  * input: touchscreen: focaltech_spi: Don't ignore firmware files
  * input: touchscreen: Add support for FocalTech ft3658 touchscreen driver
  * input: touchscreen: Disable caf default touchscreen drivers
  * input: misc: aw8697_haptic: Add support to reload firmware for umi
  * input: misc: Add support for AWINIC AW8696 Haptic driver
  * input: fingerprint: Add support for GOODIX FOD fingerprint driver
  * input: fingerprint: Add support for GOODIX fingerprint driver
  * input: fingerprint: fpc_tee: Add dummy power_cfg sysfs
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * drivers: Add support for memory fragmentation test simulator driver
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * Merge commit '99646a7c5d9584934073bd5c49438993f44c2f31' into android11-base
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * arm64: Makefile: Remove "-z norelro" from vmlinux ldflags
  * scripts: Makefile.lib: Don't disable dtc checks
  * scripts: use python rewrite in libfdt for mkdtimg
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * techpack: data: Build high performance ipa/rmnet drivers
  * techpack: video: msm: vidc: disable decode batching feature
  * Revert "selinux: Relocate ss_initialized and selinux_enforcing to separate 4k"
  * ipa3: fix improper size checks
  * msm: ipa: Fix Makefile
  * kbuild: Remove gcc-wrapper
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arch: arm64: import Xiaomi device tree
  * add toggle for disabling newly added USB devices
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * gitignore: dont ignore dts and techpack
  * dtc: Shut up
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * qcacld-3.0: Fallback to default WCNSS config path for custom ROMs
  * qcacld-3.0: wlan_hdd_misc: Fix ini file path for oos11
  * qcacld-3.0: Cleanup unused driver initialization code
  * drivers: staging: qca-wifi-host-cmn: Move IPA_TCL_DATA_RING_IDX definition out of conditional statement
  * qcacld-3.0: Disable build tagging.
  * qcacld-3.0: Only call hdd_debugfs_process_mib_stats if debugfs is enabled.
  * qcacld-3.0: qca6390_defconfig: Tone down debugging.
  * qcacld-3.0: qca6390_defconfig: Enable power debug.
  * qcacld-3.0: qca6390_defconfig: Enable desc debug check.
  * qcacld-3.0: qca6390_defconfig: Enable multi-page allocation.
  * qcacld-3.0: Fix regulatory domain country names.
  * qcacld-3.0: Nuke rx_wakelock code entirely.
  * qcacld-3.0: Defer HDD initialization.
  * qcacld-3.0: Discard wlan_boot sysfs code on !CONFIG_MODULES.
  * qcacld-3.0: Initialize variables to avoid errors during compilation.
  * qcacld-3.0: Do not manually re-enable -Wmaybe-uninitialized.
  * qcacld-3.0: Always force user build.
  * qcacld-3.0: Nuke Kconfig-based configuration entirely.
  * drivers: staging: Include qcacld-3.0 source
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/video-driver into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/display-drivers into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/camera-kernel into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge tag 'LA.UM.9.12.r1-13300-SMxx50.QSSI12.0' of https://source.codeaurora.org/quic/la/kernel/msm-4.19 into LA.UM.9.12.r1-13300-SMxx50.QSSI12.0
  * Merge 8eaf52d7f00ce80a35524f68b0339ce9a8ed55c2 on remote branch
  * Merge "USB: gadget: f_uvc: Enable required controls for CT and PU"
  * Merge "wait: add wake_up_sync()"
  * Merge "fuse: give wakeup hints to the scheduler"
  * Merge "serial: msm_geni_serial: Reduce stale delay added in stop_rx_sequencer"
  * serial: msm_geni_serial: Reduce stale delay added in stop_rx_sequencer
  * Merge "usb: f_mtp: Limit MTP Tx req length to 16k for ChipIdea"
  * Merge "msm: ipa: Fix to free up all pending EOB pages"
  * usb: f_mtp: Limit MTP Tx req length to 16k for ChipIdea
  * fuse: give wakeup hints to the scheduler
  * wait: add wake_up_sync()
  * msm: ipa: Fix to free up all pending EOB pages
  * Add support for new HSP version
  * Merge "msm: ipa3: increasing the uC interrupt timeout value"
  * Merge "msm: synx: acquire ref of synx handle in each function"
  * i2c: i2c-qcom-geni: Change the high and low time for SCL
  * msm: ipa3: increasing the uC interrupt timeout value
  * Merge "soc: qcom: Set QOS only to silver cluster"
  * Merge "msm: synx: fix synx_release_core race condition"
  * msm: kgsl: Fix out of bound write in adreno_profile_submit_time
  * Merge "soc: qcom: smem: Update size of legacy partition"
  * soc: qcom: Set QOS only to silver cluster
  * Merge "dwc3-msm: Move override usb speed functionality outside edev check"
  * dwc3-msm: Move override usb speed functionality outside edev check
  * msm: synx: acquire ref of synx handle in each function
  * msm: synx: fix synx_release_core race condition
  * soc: qcom: smem: Update size of legacy partition
  * USB: gadget: f_uvc: Enable required controls for CT and PU
