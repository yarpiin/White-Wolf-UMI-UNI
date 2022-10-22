
n.n.n / 2022-10-22
==================

  * defconfig regen
  * block: cfq-iosched: Port samsung optimizations from SM-N986B
  * ANDROID: block: cfq: Disable logging if trace is not enabled
  * cfq: clear queue pointers from cfqg after unpinning them in cfq_pd_offline
  * block: set slice_idle to 0 on cfq
  * cfq-iosched: fix the setting of IOPS mode on SSDs
  * Apply tunings from redHat[1] for faster storage.
  * block: Make CFQ default to IOPS mode on SSDs
  * block: cfq-iosched: Micro optimize
  * ANDROID: softirq: Refine RT defer softirq Do not defer softirq processing when RT throttling. Otherwise softirq process would be deferred indefinitely.
  * BACKPORT: sched/fair: Load balance aggressively for SCHED_IDLE CPUs
  * BACKPORT: sched/fair: Make sched-idle CPU selection consistent throughout
  * BACKPORT: sched/fair: Fall back to sched-idle CPU if idle CPU isn't found
  * cpuidle: lpm-levels: Only cancel the bias timer when it's used
  * Tweak cpuidle governor to enter deep C state faster
  * cpuidle: lpm-levels: Allow exit latencies equal to target latencies
  * cpufreq_schedutilX: Target loads implementation Target load. Lower values result in higher CPU speeds.
  * UPSTREAM: cpufreq: schedutil: restore cached freq when next_f is not changed
  * cpufreq: schedutil: Cleanup all iowait_boost code
  * timekeeping: Keep the tick alive when CPUs cycle out of s2idle
  * msm: kgsl: Wake GPU upon receiving an ioctl rather than upon touch input
  * msm: kgsl: Increase worker thread priority
  * clk: qcom: mdss: Fix tons of memory leaks
  * power / suspend: Use s2idle by default
  * kernel: power: Entirely checkout to msm-5.4
  * kernel: power: Checkout to msm-5.4
  * drivers: soc: qcom: Checkout to msm-5.4
  * soc/qcom: Fix errors
  * soc: qcom: qmi_interface: Parse all control packets
  * soc: qcom: qmi: Return EPROBE_DEFER if no address family
  * soc: qcom: qmi_interface: Protect sk_user_data with RCU
  * soc/qcom: Checkout to 5.4 state
  * rpmsg: char: Migrate to iter versions of read and write
  * rpmsg: checkout native module from msm-5.4
  * msm: camera: Remove nonsensical register reads in cam_vfe_fe_reg_dump()
  * alarmtimer: Don't fail on wakeup
  * defconfig: Disable ARM64 PAN emulation
  * defconfig: Disable support for SVE
  * defconfigs: Disable Qualcomm Thermal Limiter
  * drivers: thermal: limits-dcvs: Always build driver
  * block: Do not wake the request CPU if idle
  * drivers: video: Optimized Console FrameBuffer for upto 70% increase in Performance
  * msm: kgsl: Reduce latency while processing ioctls
  * mm: Increase ratelimit pages value
  * sched/fair: Switch sched scaling to linear
  * cpufreq: schedutil: Inline `sugov_policy_free()`
  * fs: default to noatime
  * fs: sync: Avoid calling fdget without fdput When adding fsync support, we check for fsync_enabled() in several cases but it appears that we should fdput() after fdget() but the current code just check for fsync_enabled and directly return in some cases after calling fdget().
  * fs: fsync on/off support
  * defconfigs: adjust cmdline for faster boot
  * cpufreq: schedutil: fix check for stale utilization values
  * ARM64: kona/lito: Optimize FOD HBM to fix fod flashing
  * touchscreen: fts_521: Fix strict-prototypes errors
  * touchscreen: goodix_driver_gt9886: Fix strict-prototypes errors
  * msm: adsprpc: Fix race condition in internal_control
  * techpack: display: temporary disable DC dimming when in fod_hbm mode
  * techpack: display: temporary disable DC dimming when in hbm mode
  * techpack: display: adapt exposure adjustment methods to sm8250
  * techpack: display: Let exposure adjustment switchable via sysfs
  * techpack: display: Introduce exposure adjustment driver
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * cnss2: Avoid race condition between time sync and system suspend
  * cnss2: Make sure the write to remap window register take effect
  * touchscreen: nt36672c: Return 0 if allocation fails in test_buff_init
  * touchscreen: nt36672c: Fix unable to set xiaomi touch interface mode
  * touchscreen: goodix_driver_gt9886: Fix memory leak in goodix_cfg_bin_proc
  * input: misc: aw8697: Fix memory leak in aw8697_file_write
  * kernel: time: reduce ntp wakeups
  * scripts/Makefile.lib: Speed up build process
  * cpufreq: Ensure the minimal frequency is lower than the maximal frequency
  * power_supply: don't allocate attrname
  * regulator: Retry read and write i2c operations on failure
  * arm64: dts: kona: Switch to step-wise thermal governor
  * BACKPORT: msm: kgsl: Add support for KGSL_PROP_VK_DEVICE_ID
  * BACKPORT: msm: kgsl: Add a property to query gpu model name
  * BACKPORT: msm: kgsl: Add support to get gpu model from device tree
  * rmnet_ipa: Fix netdev watchdog triggering on suspend
  * msm: ipa: Do not acquire wakelocks
  * arm64: dts: umi: optimise battery charging further
  * mbcache: Speed up cache entry creation
  * arm64: select HAVE_MOVE_PMD for faster mremap
  * mm: speed up mremap by 20x on large regions
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
  * sched: fix issue of cpu freq running at max always
  * ashmem: Adapt building on msm-4.19
  * ashmem: Rewrite to improve clarity and performance
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * cpufreqX: schedutil: Use kobject release() method to free sugov_tunables
  * arch: dts: Throttle charging speed and Optimise charging current
  * arm64: lib: Memory utilities optimization
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * setlocalversion: Never append a "+" to localversion
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
  * add yarpiins build stuff
  * gitignore: add out directory
  * kernel: update sm8250-pref_defconfig
  * kernel: Use sm8250-perf_defconfig for /proc/config.gz
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data
  * cas_defconfig: Sync it up with UM
  * cmi_defconfig: Sync it up with UMI
  * kernel_headers: Add missing kernel headers
  * Android.bp: allow system to use generated kernel headers
  * scsi: ufs: Guard `ufs_qcom_parse_lpm` for UMI
  * techpack: display: sde: encoder: remove intf_frame_count reading
  * techpack: display: track real backlight value
  * techpack: display: msm: sde: Increase sde stage to map zpos changes
  * drm: msm: handle more scenarios when getting fod dim alpha
  * drm/msm: fix brightness level mapping
  *  soc: qcom: smp2p_sleepstate: Add suspend delay
  * uapi: sde_drm: Define FOD_PRESSED_LAYER_ZORDER
  * techpack: display: msm: notify sysfs for fod ui changes
  * techpack: display: msm: implement fod handling
  * techpack: display: msm: dsi: add dsi_panel_get_backlight
  * techpack: display: msm: dsi: add get_main_display
  * techpack: display: msm: sde: translate zpos property to fod property
  * techpack: display: msm: dsi: create empty attributes group
  * ARM64: configs: umi: Switch to Westwood as default TCP congestion algorithm
  * ARM64: configs: umi: disable CONFIG_ZRAM_WRITEBACK
  * techpack: camera: fd: Properly fix double add into list
  * msm: camera: ife: Do not wait for timeout for config done completion
  * ANDROID: GKI: power_supply: Add PROP_MOISTURE_DETECTION_ENABLED
  * smb5-lib: clear moisture_detected and icl vote
  * smb5-lib: Use mutex instead of spin lock for moisture_detection_enable
  * qpnp-smb5: check lpd_disable before moisture detect
  * qpnp-smb5: Add support for PROP_MOISTURE_DETECTION_ENABLED
  * drivers: thermal: cleanup thermal implementation
  * BACKPORT: dfc_qmi: Honor tcp ancillary bit even when there is no change in grant
  * BACKPORT: soc: qcom: smp2p_sleepstate: Add support for multiple clients
  * BACKPORT: soc: qcom: smp2p_sleepstate: Fix compilation warning
  * BACKPORT: soc: qcom: smp2p: Fix possible smp2p entry double free
  * BACKPORT: soc: qcom: smp2p: Add support for suspend to disk
  * BACKPORT: soc: qcom: smp2p: Log unchanged entries
  * qmi_rmnet: Make powersave workqueue unbound and freezable
  * dfc: not using alarm timer
  * Revert "dfc: bearer based QMAP powersave"
  * BACKPORT: page_alloc: consider highatomic reserve in watermark fast
  * techpack: camera: Import xiaomi changes from psyche-r-oss
  * techpack: camera: cam_fd: fix double add into list
  * ARM: dts: msm: Update to LA.UM.9.12.r1-14000-SMxx50.0
  * ARM: dts: msm: Update to LA.UM.9.12.r1-13800-SMxx50.0
  * devicetree: Fast-Forward to LA.UM.9.12.r1-13600-SMxx50.0
  * cpufreq_times: Add procfs node to get per app system statistics
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data
  * sysfs: Add sysfs_emit and sysfs_emit_at to format sysfs output
  * msm: camera: Fix memory leak in cam_res_mgr_probe()
  * techpack: camera: Fix memory leak
  * msm-camera: Unmap secure buffers in secure usecase
  * msm: cleanup: union gsi_wdi2_channel_scratch2_reg
  * msm: cleanup: struct gsi_mhi_channel_scratch
  * msm: cleanup: union gsi_evt_scratch
  * msm: cleanup: union gsi_wdi_channel_scratch3_reg
  * msm: cleanup: union gsi_channel_scratch
  * Revert "ANDROID: kbuild: limit LTO inlining"
  * scsi:Fix block layer elevator stuck issue
  * debug: fix pcie device wakeup reason is unknown issue
  * ARM64: configs: enable CONFIG_WIREGUARD
  * ARM64: configs: umi: Disable Richwave RTC6226 FM Radio Receiver support
  * ARM64: configs: umi: Enable ChaCha20Poly1305 for the usage of IPsec
  * ANDROID: fully revert ANDROID_PARANOID_NETWORK
  * ANDROID: revert all xt_qtaguid stuff
  * qcacld-3.0: Do not allow any wakelocks to be held
  * binder: Fix log spam caused by interrupted waits
  * arm64: debug: Separate debug hooks based on target exception level
  * msm:ADSPRPC: Fix to avoid Use after free in fastrpc_internal_munmap
  * ARM: dts: Disable debug monitors by default
  * ARM: dts: Disable IRQ debugging
  * drm/msm/sde: Cache register values when performing clock control
  * adreno_tz: Fix GPU target frequency calculation for high refresh rates
  * input: fpc_tee: Allow filtering of key events
  * thermal: core: Workaround for xiaomi thermal mitigation
  * thermal: Add support for new xiaomi thermal message nodes
  * drivers: thermal: Don't qualify thermal polling as high priority
  * LPD: notify the usb hal to acquire the moisture status
  * mm: compaction: Use MI DRM notifier API
  * mm: compaction: Run ZRAM compaction on automatic compaction
  * mm: compaction: Fix bad logging.
  * mm: compaction: Add automatic compaction mechanism.
  * workqueue: Implement delayed_work_busy().
  * mm: zram: skip readahead for sync io pages
  * ARM64: configs: umi: Enable zram-writeback support
  * mm: zram: fix build error when zram wb disabled
  * mm: zram: fix swapcached issue on Zram Writeback
  * zram: Enable zRAM deduplication by default
  * block: genhd: add 'groups' argument to device_add_disk
  * drivers: zram_drv: Expose zram_compact() for zram0
  * zram: Do not subtract deduplicated data from compressed size when freeing pages
  * zram: off by one in read_block_state()
  * zram_drv: allow reclaim on bio_alloc
  * zram: replace fsync_bdev with sync_blockdev
  * zram: fix race between zram_reset_device() and disksize_store()
  * zram: switch to 64-bit hash for dedup
  * bdi: remove BDI_CAP_SYNCHRONOUS_IO
  * zram: defer bvec assignment
  * zram: do not writeback deduped pages
  * zram: trim the backing device upon registration
  * zram: remove writeback_limit function
  * zram: remove incompressible page handling
  * zram: show deduped status in debugfs
  * zram: enable dedup by default
  * zram: use xxhash instead of jhash in dedup
  * zram: fix race condition while returning zram_entry refcount
  * zram: compare all the entries with same checksum for deduplication
  * zram: make deduplication feature optional
  * zram: implement deduplication in zram
  * zram: introduce zram_entry to prepare dedup functionality
  * zram: fix build
  * zram: kang from v5.15
  * zsmalloc: account the number of compacted pages correctly
  * ARM64: configs: umi: Enable crypto LZ4
  * lz4: remove unused functions
  * lz4: staticify functions
  * lz4_decompress: declare LZ4_decompress_safe_withPrefix64k static
  * lib/lz4: explicitly support in-place decompression
  * lz4: fix kernel decompression speed
  * lib/lz4/lz4_decompress.c: document deliberate use of `&'
  * lz4: do not export static symbol
  * lib/lz4: update LZ4 decompressor module
  * data-kernel: rmnet: shs: Fix consistent divide-by-zero when updating stats
  * arm64: Inline the spin lock function family
  * ARM64: configs: umi: Disable unused errata
  * ARM64: configs: umi: Enable userspace CNTVCT_EL0 access for vDSO
  * BACKPORT: disp: msm: sde: increase kickoff timeout for doze usecase
  * Makefile: Use llvm ar and nm from path if available
  * cnss: Do not mandate TESTMODE for netlink driver
  * net: Allow BPF JIT to compile without module support
  * ARM64: configs: umi: Disable some unuse drivers
  * ARM64: configs: umi: Disable stability debug configs
  * ARM64: configs: umi: Enable jump label
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
  * drivers: media: cvp: Fix -Wvoid-pointer-to-int-cast
  * drivers: cpuidle: Remove unused update_ipi_history
  * video: hfi_iris2: Fix -Wpointer-to-int-cast
  * ARM64: configs: umi: Disable QHEE kernel memory protection
  * ARM64: configs: umi: Enable power efficient workqueues
  * ARM64: configs: umi: Increase vmstat interval to 20 seconds
  * mm: add Kconfig interface for vmstat interval
  * cpuidle: Do not select menu and ladder governors
  * scsi: ufs: disable clock scaling
  * rpmsg: glink: Remove IRQF_NO_SUSPEND
  * mailbox: msm_qmp: Remove IRQF_NO_SUSPEND
  * soc: qcom: smp2p: Remove IRQF_NO_SUSPEND
  * ARM64: configs: Disable serial console support
  * ARM64 configs: umi: Disable qti core control and sched autogroup
  * ARM64: configs: umi: Disable some debug drivers
  * cpuidle: lpm-levels: Remove debug event logging
  * binder: Fix log spam caused by interrupted waits
  * ARM64: configs: Bringup custom defconfig for UMI
  * ASoC: pcm: Add 24bit playback audio support
  * msm: kgsl: introduce CONFIG_CORESIGHT_ADRENO.
  * GKI: ARM: dts: msm: disable coresight for kona/lito
  * GKI: hwtracing: Add a driver for disabling coresight clocks
  * tcp: Enable ECN negotiation by default
  * tcp_bbr: centralize code to set gains
  * block: disable I/O stats accounting by default
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
  * subsystem_restart: Always performs soft resets when subsystems crash
  * smb5: report fast charging when a proprietary charger is attached
  * PM / sleep: Add sysfs to log device wakeup time
  * soc: qcom: Add config to reduce SMP2P sleepstate wakeup time to 100ms
  * net: cnss2: Avoid entering L1 state while mhi fast resuming
  * block: blk-wbt: Check for WBT request before throttling down
  * proc: some optimization for reclaim
  * drivers: power: Import xiaomi modifications from elish-r-oss
  * input: aw8697_haptic: Add support to retry f0 register check
  * input: aw8697_haptic: Refactor for sanity and consistency
  * input: aw8697_haptic: Remove duplicated aw8697_i2c_read usage in aw8697_irq
  * input: aw8697_haptic: Add mutex lock for aw8697_haptic_rtp_init protection
  * drivers: aw8697: Fix -Wpointer-to-int-cast
  * ARM64: configs: Enable support for UAS storage devices
  * ARM64: configs: Set CONFIG_HZ to 300
  * input: touchscreen: xiaomi: Prevent unnecessary input sync
  * drivers: Remove xiaomi disable LPM and IRQ boost modifcations
  * firmware: Upgrade focaltech_ts FW from MIUI 21.4.21
  * input: touchscreen: focaltech_spi: Upgrade ft3658 k11 firmware
  * power: supply: qcom: Disable debug masks
  * input: aw8697_haptic: Disable Debugging
  * drivers: Remove xiaomi early fingerprint wakeup optimization
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
  * ARM64: configs: Initial copy of vendor/kona-perf_defconfig
  * net: Add support for QTI Smart Link Aggregation driver
  * init: do_mounts: Increase name value when block device is detected
  * fs: pstore: Add support to capture last_kmsg
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
  * soc: qcom: Add soc information for xiaomi SM8250 and SM7250 devices
  * soc: qcom: service-locator: Enlarge locator service timeout value
  * scsi: ufs: Address PA_HIBER8TIME fix for samsung KLUFG8RHDA-B2D1
  * scsi: ufs: increase power control timeout
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
  * input: Add support to dump kernel logs by long pressing the buttons
  * kernel: Implement sysfs to get powerup restart reasons
  * input: touchscreen: nt36xxx: Disable Debugging
  * input: touchscreen: fts_521: Disable Debugging
  * input: touchscreen: fts_521: Disable touch debugfs support if !DEBUG_FS
  * input: touchscreen: nt36672c: Disable Debugging
  * input: touchscreen: xiaomi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Debugging
  * input: touchscreen: focaltech_spi: Disable Debugging
  * input: touchscreen: focaltech_touch: Disable Production test module
  * input: touchscreen: focaltech_spi: Disable Production test module
  * drivers: input: touchscreen: goodix_driver_gt9886: Implement double_tap node
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
  * input: fingerprint: Add support for FPC TEE fingerprint driver
  * input: fingerprint: Add support for FPC FOD fingeprint driver
  * drivers: input: Add support for fingerprint drivers
  * techpack: Build high performance rmnet optimization drivers
  * techpack: camera: Import minimal xiaomi camera modifications
  * techpack: display: Import xiaomi display drivers modifications
  * techpack: audio: Correct symlinks
  * techpack: audio: Import xiaomi audio drivers modifications
  * techpack: audio: ASoC: Add audio support for lito
  * treewide: Remove all Android.mk files
  * ARM64: Add Xiaomi SM8250 and SM7250 plaform configuration
  * fs: exfat: Add support for building inside kernel
  * Add 'fs/exfat/' from commit '034f47d607c441e7fb5be3eb71d7a289ef76bc0c'
  * arch: arm64: dts: Exclude standard dts if vendor dts exists
  * arm64: Don't build legacy QCOM DTS.
  * ARM: dts: Build board specific dtbo overlays
  * ARM64: dts: qcom: Import kona mtp devicetree
  * ARM64: dts: Fix CCI timeout for OIS on xiaomi devices
  * Revert "ARM64: dts: Remove address from node-name for reserved-mem regions"
  * Revert "ARM64: dts: Correct SLPI/ADSP/SPSS/CDSP memory regions on LMI/CMI/UMI"
  * Add 'arch/arm64/boot/dts/vendor/' from commit 'bd2d6b0afa1f8aba19e41ce3bc29c16745595efa'
  * BACKPORT: scripts/dtc: only append to HOST_EXTRACFLAGS instead of overwriting
  * BACKPORT: dtc: Use pkg-config to locate libyaml
  * BACKPORT: treewide: prefix header search paths with $(srctree)/
  * BACKPORT: of: add dtc annotations functionality to dtx_diff
  * BACKPORT: kbuild: consolidate Devicetree dtb build rules
  * scripts: use aosp python mkdtboimg for cmd_mkdtimg
  * scripts: Makefile: suppress DTC compiler warnings
  * dtbo.img: build device tree overlay partition image
  * build-dtbo: Support base dtbs which located in foreign folder
  * Revert "scripts: gcc-wrapper: Use wrapper to check compiler warnings"
  * scripts/dtc: Update to upstream version v1.5.0-30-g702c1b6c0e73
  * scripts/dtc: Update to upstream version v1.5.0-23-g87963ee20693
  * scripts/dtc: Update to upstream version v1.4.7-57-gf267e674d145
  * scripts/dtc: Update to upstream version v1.4.7-14-gc86da84d30e4
  * scripts/dtc: Add yamltree.c to dtc sources
  * Android.bp: Namespace it
  * Android: Add empty Android.mk file
  * qcacld-3.0: Fix race during peer deletion in roaming scenario
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
  * Makefile: Set correct techpack audio header install directory
  * Makefile: Add missing dst for techpack header include
  * techpack: audio: makefile: do not export all the variables
  * Merge tag 'LA.UM.9.12.r1-14800-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/video-driver into LA.UM.9.12.r1-XXXX-SMxx50.QSSI13.0
  * Merge tag 'LA.UM.9.12.r1-14800-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/display-drivers into LA.UM.9.12.r1-XXXX-SMxx50.QSSI13.0
  * Merge tag 'LA.UM.9.12.r1-14800-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/camera-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI13.0
  * Merge tag 'LA.UM.9.12.r1-14800-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI13.0
  * Merge tag 'LA.UM.9.12.r1-14800-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI13.0
  * Merge tag 'LA.UM.9.12.r1-14800-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.r1-XXXX-SMxx50.QSSI13.0
  * Merge tag 'LA.UM.9.12.r1-14800-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.r1-XXXX-SMxx50.QSSI13.0
  * Merge tag 'LA.UM.9.12.r1-14800-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI13.0
  * gitignore: do not ignore techpack and vendor device tree
  * Merge branch 'linux-4.19.y' of https://github.com/jaegeuk/f2fs-stable into LA.UM.9.12.r1-XXXX-SMxx50.QSSI13.0
  * Merge tag 'LA.UM.9.12.r1-14800-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI13.0
  * initial commit
  * Merge a4341e9d9e9fa666a84167d89e2f5860b42ed6fc on remote branch
  * f2fs: change to use atomic_t type form sbi.atomic_files
  * f2fs: account swapfile inodes
  * f2fs: allow direct read for zoned device
  * f2fs: support recording errors into superblock
  * f2fs: support recording stop_checkpoint reason into super_block
  * f2fs: remove the unnecessary check in f2fs_xattr_fiemap
  * f2fs: introduce cp_status sysfs entry
  * f2fs: fix to detect corrupted meta ino
  * f2fs: fix to account FS_CP_DATA_IO correctly
  * f2fs: code clean and fix a type error
  * f2fs: add "c_len" into trace_f2fs_update_extent_tree_range for compressed file
  * f2fs: fix to do sanity check on summary info
  * f2fs: fix to do sanity check on destination blkaddr during recovery
  * f2fs: let FI_OPU_WRITE override FADVISE_COLD_BIT
  * f2fs: fix race condition on setting FI_NO_EXTENT flag
  * f2fs: remove redundant check in f2fs_sanity_check_cluster
  * f2fs: add static init_idisk_time function to reduce the code
  * f2fs: fix typo
  * f2fs: fix wrong dirty page count when race between mmap and fallocate.
  * f2fs: use COMPRESS_MAPPING to get compress cache mapping
  * f2fs: return the tmp_ptr directly in __bitmap_ptr
  * Merge "power: qpnp-fg-gen4: Improve SDAM corruption check"
  * Merge "qcom/l2cache_counters: Add support to check available CPUS"
  * Merge "cpu-topology: Don't error on more than CONFIG_NR_CPUS CPUs in device tree"
  * qcom/l2cache_counters: Add support to check available CPUS
  * Merge "FROMGIT: arm64: fix oops in concurrently setting insn_emulation sysctls"
  * Merge "media:uvc: supports dynamic setting of urb configuration"
  * FROMGIT: arm64: fix oops in concurrently setting insn_emulation sysctls
  * media:uvc: supports dynamic setting of urb configuration
  * Merge "msm: camera: Increase the total number of camera ID's supported"
  * Merge "FROMLIST: binder: fix UAF of ref->proc caused by race condition"
  * Merge "usb: gadget: rndis: prevent integer overflow in rndis_set_response()"
  * Merge "soc: qcom: socinfo: Add soc information for Khaje IOT"
  * msm: camera: Increase the total number of camera ID's supported
  * cpu-topology: Don't error on more than CONFIG_NR_CPUS CPUs in device tree
  * usb: gadget: rndis: prevent integer overflow in rndis_set_response()
  * FROMLIST: binder: fix UAF of ref->proc caused by race condition
  * Merge "msm_perf: check cpu_possible to improve stability"
  * msm_perf: check cpu_possible to improve stability
  * msm: adsprpc: Validate the CID
  * Merge 367074bf05bc4eecfdb4ee3f356f34312efc0d00 on remote branch
  * f2fs: simplify code in f2fs_prepare_decomp_mem
  * f2fs: replace logical value "true" with a int number
  * f2fs: increase the limit for reserve_root
  * f2fs: complete checkpoints during remount
  * f2fs: flush pending checkpoints when freezing super
  * f2fs: remove gc_urgent_high_limited for cleanup
  * f2fs: fix wrong continue condition in GC
  * f2fs: LFS mode does not support ATGC
  * soc: qcom: socinfo: Add soc information for Khaje IOT
  * Merge "soc: qcom: socinfo: Add soc information for KhajeG"
  * soc: qcom: socinfo: Add soc information for KhajeG
  * msm: ipa: Set the logbuf NULL after destroy
  * Merge "f2fs: guarantee to write dirty data when enabling checkpoint back"
  * Merge "msm: kgsl: use kvcalloc for ringbuffer submission"
  * Merge "f2fs: flush data when enabling checkpoint back"
  * power: smb1398-charger: Fix wrong IIN value in register
  * misc: updated log output for nordic driver
  * Merge 227ed6d234e0d6a89c26d60abd1e424fd7b83111 on remote branch
  * Merge "cnss2: Avoid dumping PBL/SBL registers"
  * Merge "msm: adsprpc: Fix for hyp_assign_phys double call"
  * Merge "mmc: Enable Inline Crypto Engine clocks before programming key"
  * msm: adsprpc: Fix for hyp_assign_phys double call
  * Merge "defconfig: kona: Enable I210 driver for Ethernet Card"
  * msm: kgsl: use kvcalloc for ringbuffer submission
  * defconfig: kona: Enable I210 driver for Ethernet Card
  * defconfig: kona: Enable I210 driver for Ethernet Card
  * Merge "drivers: thermal: Add valid cpu check in cpu isolate driver"
  * drivers: thermal: Add valid cpu check in cpu isolate driver
  * cnss2: Avoid dumping PBL/SBL registers
  * msm: kgsl: Use consolidated power level for thermal limit
  * f2fs: use onstack pages instead of pvec
  * f2fs: intorduce f2fs_all_cluster_page_ready
  * f2fs: clean up f2fs_abort_atomic_write()
  * f2fs: handle decompress only post processing in softirq
  * f2fs: do not allow to decompress files have FI_COMPRESS_RELEASED
  * f2fs: do not set compression bit if kernel doesn't support
  * f2fs: remove device type check for direct IO
  * f2fs: fix null-ptr-deref in f2fs_get_dnode_of_data
  * f2fs: revive F2FS_IOC_ABORT_VOLATILE_WRITE
  * f2fs: fix fallocate to use file_modified to update permissions consistently
  * vfs: introduce file_modified() helper
  * f2fs: fix to do sanity check on segment type in build_sit_entries()
  * f2fs: obsolete unused MAX_DISCARD_BLOCKS
  * f2fs: fix to avoid use f2fs_bug_on() in f2fs_new_node_page()
  * f2fs: fix to remove F2FS_COMPR_FL and tag F2FS_NOCOMP_FL at the same time
  * f2fs: introduce sysfs atomic write statistics
  * f2fs: don't bother wait_ms by foreground gc
  * f2fs: invalidate meta pages only for post_read required inode
  * f2fs: allow compression of files without blocks
  * f2fs: fix to check inline_data during compressed inode conversion
  * f2fs: fix to invalidate META_MAPPING before DIO write
  * f2fs: adjust zone capacity when considering valid block count
  * f2fs: enforce single zone capacity
  * Revert "f2fs: handle decompress only post processing in softirq"
  * Revert "f2fs: run GCs synchronously given user requests"
  * Merge 523e8bfd6e7680cc5b3916fac9bad18bdf9e5d20 on remote branch
  * msm: kona_defconfig: Added the flag for ICE Driver as required by FDE
  * mmc: Enable Inline Crypto Engine clocks before programming key
  * gup: document and work around "COW can break either way" issue
  * power: qpnp-fg-gen4: Improve SDAM corruption check
  * Merge "net: usb: ax88179_178a: support setting mac to eeprom"
  * Merge "gup: document and work around "COW can break either way" issue"
  * misc: updates to controller's driver
  * Merge "power: qpnp-fg-gen4: Update msoc on every FVSS soc update"
  * gup: document and work around "COW can break either way" issue
  * Merge "msm: ipa3: add check in odl pipe cleanup"
  * Merge "ipa3: Changes to call disable datapth API before stop channel"
  * Merge "msm: disp: dp: add aux dp support by usb mode"
  * Merge "tz_log : use file->private_data to get tz_id for debugfs"
  * Merge "msm: adsprpc: Handle out of bounds access"
  * Merge "HID: check for valid USB device for many HID drivers"
  * power: qpnp-fg-gen4: Update msoc on every FVSS soc update
  * ipa3: Changes to call disable datapth API before stop channel
  * tz_log : use file->private_data to get tz_id for debugfs
  * msm: adsprpc: Handle out of bounds access
  * net: usb: ax88179_178a: support setting mac to eeprom
  * msm: disp: dp: add aux dp support by usb mode
  * Merge 34bb42eb71428965dc411f8bbc691e8280c84259 on remote branch
  * msm: ipa3: add check in odl pipe cleanup
  * f2fs: guarantee to write dirty data when enabling checkpoint back
  * f2fs: flush data when enabling checkpoint back
  * Merge "FROMGIT: cgroup: Use separate src/dst nodes when preloading css_sets for migration"
  * msm: ipahal: modify parameter from eq_bitfield[i] to i
  * HID: check for valid USB device for many HID drivers
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
  * f2fs: remove redundant code for gc condition
  * f2fs: handle decompress only post processing in softirq
  * f2fs: introduce memory mode
  * f2fs: initialize page_array_entry slab only if compression feature is on
  * f2fs: optimize error handling in redirty_blocks
  * f2fs: do not skip updating inode when retrying to flush node page
  * misc: updates to controller's driver
  * HID: add hid_is_usb() function to make it simpler for USB detection
  * Merge "lib/iov_iter: initialize "flags" in new pipe_buffer"
  * Merge "UPSTREAM: USB: gadget: validate interface OS descriptor requests"
  * Merge "UPSTREAM: net/packet: rx_owner_map depends on pg_vec"
  * Merge "UPSTREAM: mmc: block: fix read single on recovery logic"
  * Merge "defconfig: Enable KinecticsXR Nordic chip for SKU4"
  * Merge "smcinvoke : file private data validation which is sent by userspace"
  * f2fs: do not count ENOENT for error case
  * f2fs: run GCs synchronously given user requests
  * f2fs: attach inline_data after setting compression
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
  * Merge "defconfig: msm: Add initial defconfig for Bengal-lite"
  * Merge "soc: qcom: Add retry mechanism to encryption related Secure Monitor calls"
  * smcinvoke : file private data validation which is sent by userspace
  * defconfig: msm: Add initial defconfig for Bengal-lite
  * Merge 0bfc6c856bf196fe10ddb9b249543a9283054fce on remote branch
  * f2fs: attach inline_data after setting compression
  * mmc: sdhci-msm: Revert clear tuning done flag while hs400 tuning
  * f2fs: fix to tag gcing flag on page during file defragment
  * f2fs: replace F2FS_I(inode) and sbi by the local variable
  * f2fs: add f2fs_init_write_merge_io function
  * f2fs: avoid unneeded error handling for revoke_entry_slab allocation
  * f2fs: allow compression for mmap files in compress_mode=user
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
  * f2fs: fix typo in comment
  * f2fs: make f2fs_read_inline_data() more readable
  * Merge "msm: kgsl: Remove 'fd' dependency to get dma_buf handle"
  * add controller's drivers
  * f2fs: fix to do sanity check for inline inode
  * Merge 7d6b043520ec4d7d9574ce49de0357bb5713dd92 on remote branch
  * f2fs: don't use casefolded comparison for "." and ".."
  * f2fs: do not stop GC when requiring a free section
  * msm: kgsl: Remove 'fd' dependency to get dma_buf handle
  * msm: adsprpc: Fix compat ioctl invoke failure
  * f2fs: keep wait_ms if EAGAIN happens
  * f2fs: introduce f2fs_gc_control to consolidate f2fs_gc parameters
  * f2fs: reject test_dummy_encryption when !CONFIG_FS_ENCRYPTION
  * f2fs: kill volatile write support
  * f2fs: change the current atomic write way
  * f2fs: don't need inode lock for system hidden quota
  * diag: Update log and event mask code ranges
  * smp: Fix smp_call_function_single_async prototype
  * Makefile: Move -Wno-unused-but-set-variable out of GCC only block
  * msm: ipa3: Avoid device crash while SSR
  * f2fs: stop allocating pinned sections if EAGAIN happens
  * f2fs: skip GC if possible when checkpoint disabling
  * f2fs: give priority to select unpinned section for foreground GC
  * f2fs: fix to do sanity check on total_data_blocks
  * f2fs: fix deadloop in foreground GC
  * f2fs: fix to do sanity check on block address in f2fs_do_zero_range()
  * f2fs: fix to avoid f2fs_bug_on() in dec_valid_node_count()
  * f2fs: write checkpoint during FG_GC
  * f2fs: fix to clear dirty inode in f2fs_evict_inode()
  * f2fs: extend stat_lock to avoid potential race in statfs
  * f2fs: avoid infinite loop to flush node pages
  * f2fs: use flush command instead of FUA for zoned device
  * f2fs: remove WARN_ON in f2fs_is_valid_blkaddr
  * msm: kgsl: Use copy_struct_from_user() helper
  * Merge "msm: ADSPRPC: Update unsigned pd support on cDSP from kernel"
  * Merge "BACKPORT: dmabuf: fix use-after-free of dmabuf's file->f_inode"
  * Merge "lib: introduce copy_struct_from_user() helper"
  * Merge "msm: synx: release reference holding the lock"
  * qcom: genoa_extcon_notifier: Add genoa extcon notifier driver
  * msm: ADSPRPC: Update unsigned pd support on cDSP from kernel
  * Merge "msm: kgsl: Remove unnecessary NULL checks"
  * Merge "msm: kgsl: Assign a default value to the variable ret"
  * dfc: bearer based QMAP powersave
  * Merge "fbdev: msm: check for valid fence before using objects"
  * f2fs: replace usage of found with dedicated list iterator variable
  * f2fs: Remove usage of list iterator pas the loop for list_move_tail()
  * f2fs: fix dereference of stale list iterator after loop body
  * f2fs: fix to do sanity check on inline_dots inode
  * f2fs: introduce data read/write showing path info
  * f2fs: remove unnecessary f2fs_lock_op in f2fs_new_inode
  * f2fs: don't set GC_FAILURE_PIN for background GC
  * f2fs: check pinfile in gc_data_segment() in advance
  * f2fs: should not truncate blocks during roll-forward recovery
  * f2fs: fix wrong condition check when failing metapage read
  * f2fs: keep io_flags to avoid IO split due to different op_flags in two fio holders
  * f2fs: remove obsolete whint_mode
  * Merge "sched: Improve the scheduler"
  * BACKPORT: dmabuf: fix use-after-free of dmabuf's file->f_inode
  * Merge "sched: walt: Improve the scheduler"
  * sched: Improve the scheduler
  * lib: introduce copy_struct_from_user() helper
  * fbdev: msm: check for valid fence before using objects
  * Merge "diag: Use correct size while reallocating for hdlc encoding"
  * f2fs: pass the bio operation to bio_alloc_bioset
  * f2fs: don't pass a bio to f2fs_target_device
  * f2fs: replace congestion_wait() calls with io_schedule_timeout()
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
  * Merge 7e6242a76c9ada28629e35bc29bdc8c9950c1b59 on remote branch
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
  * Merge "msm: adsprpc: Do length check to avoid arbitrary memory access"
  * msm: adsprpc: Do length check to avoid arbitrary memory access
  * f2fs: fix to do sanity check on .cp_pack_total_block_count
  * f2fs: make gc_urgent and gc_segment_mode sysfs node readable
  * Merge "pci_iomap: fix page fault issue on vmalloc with section mapping"
  * Merge "aio: fix use-after-free due to missing POLLFREE handling"
  * f2fs: use aggressive GC policy during f2fs_disable_checkpoint()
  * f2fs: fix compressed file start atomic write may cause data corruption
  * Merge "FROMGIT: USB: gadget: bRequestType is a bitfield, not a enum"
  * f2fs: initialize sbi->gc_mode explicitly
  * msm: kgsl: Zap performance counters across context switches
  * msm: kgsl: Add a sysfs node to control performance counter reads
  * f2fs: introduce gc_urgent_mid mode
  * f2fs: compress: fix to print raw data size in error path of lz4 decompression
  * f2fs: remove redundant parameter judgment
  * Merge "aio: keep poll requests on waitqueue until completed"
  * fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs
  * Merge "sctp: add param size validation for SCTP_PARAM_SET_PRIMARY"
  * Merge "binder: use wake_up_pollfree()"
  * Merge "fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs"
  * f2fs: use spin_lock to avoid hang
  * f2fs: don't get FREEZE lock in f2fs_evict_inode in frozen fs
  * f2fs: remove unnecessary read for F2FS_FITS_IN_INODE
  * icnss: Add code to pass device configs to wlan driver
  * fix 'key ffffffe58e1b37c0 not in .data!' in  nvmem and dcvs
  * msm: kgsl: Zap performance counters across context switches
  * msm: kgsl: Add a sysfs node to control performance counter reads
  * disp: uapi: drm: Add new blend type for CAC
  * Merge "msm: npu: Avoid buffer overflow when handling get_property packet"
  * Merge "USB: uac1: Fix audio quality issues for UAC1"
  * Merge "msm_bus: Check if cldata->pdata is null before access it"
  * f2fs: introduce F2FS_UNFAIR_RWSEM to support unfair rwsem
  * f2fs: avoid an infinite loop in f2fs_sync_dirty_inodes
  * f2fs: fix to do sanity check on curseg->alloc_type
  * f2fs: fix to avoid potential deadlock
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
  * f2fs: quota: fix loop condition at f2fs_quota_sync()
  * f2fs: Restore rwsem lockdep support
  * f2fs: fix missing free nid in f2fs_handle_failed_inode
  * Merge 24aebadd512b27f8cb15a4b5ae634e104d74e183 on remote branch
  * Merge "cnss2: Handle event processing after shutdown"
  * Merge "soc: qcom: socinfo: Add soc information for KhajeP and KhajeQ"
  * Merge "i2c: i2c-msm-v2: Set frequency parameters to INT and validate"
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
  * drivers: thermal: validate cdev sysfs state request before using it
  * f2fs: add a way to limit roll forward recovery time
  * f2fs: introduce F2FS_IPU_HONOR_OPU_WRITE ipu policy
  * f2fs: adjust readahead block number during recovery
  * f2fs: fix to unlock page correctly in error path of is_alive()
  * f2fs: expose discard related parameters in sysfs
  * f2fs: move discard parameters into discard_cmd_control
  * USB: f_fs: Fix disconnect check during ongoing IO
  * Merge "msm: diag: fix copyright"
  * Merge "drivers: soc: qcom: Initialize blocking notifier as per lockdep"
  * msm: diag: fix copyright
  * Merge "diag: Validate the dci client before sending dci packet"
  * diag: Validate the dci client before sending dci packet
  * drivers: soc: qcom: Initialize blocking notifier as per lockdep
  * usb: gadget: u_audio: fix race condition on endpoint stop
  * usb: gadget: u_audio: Free requests only after callback
  * defconfig: Enable usb peripheral audio on SXR2130
  * Merge 1c29131bbaafca9e6ee6ed00bf330c9243c6d804 on remote branch
  * Merge "diag: Ensure dci entry is valid before sending the packet"
  * Merge "msm: kgsl: Perform cache flush on the pages obtained using get_user_pages()"
  * msm: ADSPRPC: Fix to avoid Use after free in fastrpc_init_process
  * Merge "msm: synx: fix copyright"
  * Merge "msm: ipa3: Fix to validate the NAT table entries during NAT table init"
  * clk: qcom: vdd-level: Update the vdd level for CX on Khaje
  * msm: ipa3: Fix to validate the NAT table entries during NAT table init
  * msm: synx: fix copyright
  * clk: qcom: gpucc: Update the vdd level for CX on Khaje
  * diag: Ensure dci entry is valid before sending the packet
  * f2fs: fix to enable ATGC correctly via gc_idle sysfs interface
  * f2fs: move f2fs to use reader-unfair rwsems
  * Merge "usb: pd: Process request message as soon as it is received"
  * usb: pd: Process request message as soon as it is received
  * Merge "dfc: reset tx queue"
  * f2fs: do not allow partial truncation on pinned file
  * f2fs: remove redunant invalidate compress pages
  * f2fs: Simplify bool conversion
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
  * Merge fd4cdfacba48b59775c41ed00cbc28a63ead2026 on remote branch
  * spi: spi-msm-geni: Protect from register access in suspend state
  * spi: spi-msm-geni: Keep device to suspend if PM call fails
  * Merge "regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling"
  * Merge "seq_file: disallow extremely large seq buffer allocations"
  * regulator: qcom_pm8008-regulator: Avoid deadlock in OCP handling
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
  * f2fs: don't drop compressed page cache in .{invalidate,release}page
  * f2fs: fix to reserve space for IO align feature
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
  * f2fs: fix to check available space of CP area correctly in update_ckpt_flags()
  * f2fs: support fault injection to f2fs_trylock_op()
  * f2fs: clean up __find_inline_xattr() with __find_xattr()
  * f2fs: fix to do sanity check on last xattr entry in __f2fs_setxattr()
  * f2fs: do not bother checkpoint by f2fs_get_node_info
  * f2fs: avoid down_write on nat_tree_lock during checkpoint
  * clk: qcom: debugcc: Remove gcc_pcnoc_mpu_cfg_ahb_clk for QM215
  * clk: qcom: debugcc: Remove im_sleep/xo_div4 for QM215
  * af_unix: fix garbage collect vs MSG_PEEK
  * net: split out functions related to registering inflight socket files
  * uapi: sound: add bt external sink delay parameter
  * mhi: core: Increase RDDM timeout to 350ms
  * msm: npu: remove asynchronous network execution support
  * f2fs: compress: fix potential deadlock of compress file
  * f2fs: avoid EINVAL by SBI_NEED_FSCK when pinning a file
  * f2fs: add gc_urgent_high_remaining sysfs node
  * f2fs: fix to do sanity check in is_alive()
  * f2fs: fix to avoid panic in is_alive() if metadata is inconsistent
  * f2fs: fix to do sanity check on inode type during garbage collection
  * usb: max-3421: Prevent corruption of freed memory
  * mmc: host: Add recovery_finish crypto vops
  * seq_file: disallow extremely large seq buffer allocations
  * dfc: hold wakelock while powersave timer is running
  * f2fs: avoid duplicate call of mark_inode_dirty
  * f2fs: fix remove page failed in invalidate compress pages
  * f2fs: fix the f2fs_file_write_iter tracepoint
  * f2fs: do not expose unwritten blocks to user by DIO
  * f2fs: reduce indentation in f2fs_file_write_iter()
  * f2fs: rework write preallocations
  * f2fs: compress: reduce one page array alloc and free when write compressed page
  * f2fs: show number of pending discard commands
  * diag: Avoid possible out of bound access while sending masks
  * diag: Sanitize non-hdlc pkt length against buffer capacity
  * diag: Allow DCI packets to be copied separately
  * msm: gsi: Stop Channel support when in Flow Control State
  * msm: kgsl: Signal fence only if last fence refcount was not put
  * f2fs: check nr_pages for readahead
  * msm: adsprpc: Handle UAF in process shell memory
  * Merge 82146398a44b8081ef002068af683b1f44f4b226 on remote branch
  * pci_iomap: fix page fault issue on vmalloc with section mapping
  * Merge "msm: adsprpc: Handle UAF in process shell memory"
  * media: v4l2: Allow ioctl type of "U" for video devices like usb camera
  * msm: adsprpc: Handle UAF in process shell memory
  * Merge "cnss2: Check if firmware asserts before power off for CBC"
  * Merge "net: qrtr: Use radix_tree_iter_delete to delete tx flow"
  * Merge "net: qrtr: Cleanup flow control during DEL proc"
  * cnss2: Check if firmware asserts before power off for CBC
  * Merge "net: qrtr: Cleanup flow control during remote socket release"
  * Merge "usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path"
  * usb: gadget: cdev: Add single packet and dynamic buffer support for Rx path
  * Merge "ipa: Null persistent pointers after free"
  * Merge "leds: qpnp-flash-v2: Add support for dynamic torch current update"
  * Merge "qdss_bridge: handle usb write done event"
  * net: qrtr: Use radix_tree_iter_delete to delete tx flow
  * ipa: Null persistent pointers after free
  * net: qrtr: Cleanup flow control during DEL proc
  * qdss_bridge: handle usb write done event
  * net: qrtr: Cleanup flow control during remote socket release
  * leds: qpnp-flash-v2: Add support for dynamic torch current update
  * thermal: Update copyright info for ADC_TM driver
  * net: qrtr: Converting DEL_PROC command to BYE command
  * f2fs: fix UAF in f2fs_available_free_memory
  * f2fs: invalidate META_MAPPING before IPU/DIO write
  * f2fs: support fault injection for dquot_initialize()
  * f2fs: fix incorrect return value in f2fs_sanity_check_ckpt()
  * f2fs: compress: disallow disabling compress on non-empty compressed file
  * f2fs: compress: fix overwrite may reduce compress ratio unproperly
  * f2fs: multidevice: support direct IO
  * f2fs: introduce fragment allocation mode mount option
  * f2fs: include non-compressed blocks in compr_written_block
  * f2fs: fix wrong condition to trigger background checkpoint correctly
  * f2fs: fix to use WHINT_MODE
  * f2fs: fix up f2fs_lookup tracepoints
  * f2fs: set SBI_NEED_FSCK flag when inconsistent node block found
  * f2fs: introduce excess_dirty_threshold()
  * f2fs: avoid attaching SB_ACTIVE flag during mount
  * f2fs: quota: fix potential deadlock
  * f2fs: should use GFP_NOFS for directory inodes
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
  * Revert "ANDROID: net: ipv4: sysfs_net_ipv4: Add sysfs-based knobs for controlling TCP window size"
  * Revert "RFC: ANDROID: net: ipv4: sysfs_net_ipv4: Fix TCP window size controlling knobs"
  * Merge "cnss_utils: Update the copyright years of file to 2017, 2019-2021"
  * cnss_utils: Update the copyright years of file to 2017, 2019-2021
  * Merge "clk: qcom: debugcc: Remove the gcc_camss_camnoc clocks"
  * Merge "FM: mutex changes modified"
  * f2fs: should put a page beyond EOF when preparing a write
  * f2fs: deallocate compressed pages when error happens
  * FM: mutex changes modified
  * Merge "Revert "ANDROID: net: ipv4: sysfs_net_ipv4: Add sysfs-based knobs for controlling TCP window size""
  * Merge "coresight: byte-cnter: limit error log output"
  * Merge "coresight: byte-cnter: Add ETR status check in bypass notifier"
  * Revert "qseecom: Add shmbrdige to allocate memory"
  * f2fs: enable realtime discard iff device supports discard
  * f2fs: guarantee to write dirty data when enabling checkpoint back
  * f2fs: fix to unmap pages from userspace process in punch_hole()
  * f2fs: fix unexpected ENOENT comes from f2fs_map_blocks()
  * f2fs: fix to account missing .skipped_gc_rwsem
  * f2fs: adjust unlock order for cleanup
  * f2fs: Don't create discard thread when device doesn't support realtime discard
  * Merge "serial: msm_geni_serial: Enable SW flow on in runtime suspend"
  * Merge "coresight: byte-cntr: Add ETR status check in bypass notifier"
  * Revert "ANDROID: net: ipv4: sysfs_net_ipv4: Add sysfs-based knobs for controlling TCP window size"
  * Revert "RFC: ANDROID: net: ipv4: sysfs_net_ipv4: Fix TCP window size controlling knobs"
  * ANDROID: xt_qtaguid: fix UAF race
  * coresight: byte-cntr: Add ETR status check in bypass notifier
  * clk: qcom: debugcc: Remove the gcc_camss_camnoc clocks
  * coresight: byte-cnter: limit error log output
  * coresight: byte-cnter: Add ETR status check in bypass notifier
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
  * f2fs: rebuild nat_bits during umount
  * f2fs: introduce periodic iostat io latency traces
  * f2fs: separate out iostat feature
  * f2fs: compress: do sanity check on cluster
  * f2fs: fix description about main_blkaddr node
  * f2fs: convert S_IRUGO to 0444
  * f2fs: fix to keep compatibility of fault injection interface
  * f2fs: support fault injection for f2fs_kmem_cache_alloc()
  * f2fs: compress: allow write compress released file after truncate to zero
  * Merge 3bfa948fdbfe8d379f4a253ade1676185175b066 on remote branch
  * Merge "cnss_utils: Increase unsafe channel max num for 6G"
  * cnss_utils: Increase unsafe channel max num for 6G
  * Revert "msm: adsprpc: Lock list before removing node"
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
  * Merge "f2fs: fix the periodic wakeups of discard thread"
  * dma-mapping-fast: Fix sg-list length calculation in fast_smmu_unmap_sg()
  * qseecom: Add shmbrdige to allocate memory
  * msm: kgsl: Use worker to put refcount on mem entry
  * usb: phy: qusb: Add wrapper function for phy reset
  * usb: pd: Clear vdm_tx if SVDM message is sent on SOP'
  * drivers: thermal: virtual-sensor: Add new virtual sensor for MSM8917
  * Merge "usb: phy: Reset PHY while disabling dpdm regulator"
  * defconfig: Enable CONFIG_ION_SYSTEM_HEAP for bengal
  * futex: Handle faults correctly for PI futexes
  * f2fs: fix the periodic wakeups of discard thread
  * f2fs: allow to change discard policy based on cached discard cmds
  * futex: Simplify fixup_pi_state_owner()
  * futex: Provide and use pi_state_update_owner()
  * usb: phy: Reset PHY while disabling dpdm regulator
  * futex: Replace pointless printk in fixup_owner()
  * soc: qcom: smem: Update size of legacy partition
  * Merge "USB: diag: Add check for ctxt in usb_diag_request_size()"
  * Merge "msm: ipa3: Changes to check disconnect in progress while sending data"
  * Merge "msm: adsprpc: Protect maps using map mutex"
  * USB: diag: Add check for ctxt in usb_diag_request_size()
  * driver: Fix compilation error with new sdclang 12
  * msm: ipa3: Changes to check disconnect in progress while sending data
  * msm: adsprpc: Protect maps using map mutex
  * Merge "defconfig: Enable thermal emergency poweroff delay"
  * USB: f_diag: Report Max request size as 16k for ChipIdea
  * Merge "mmc: sdhci-msm: Switch to required pad voltage for vbias bypass"
  * defconfig: Enable thermal emergency poweroff delay
  * Merge "futex: Ensure the correct return value from futex_lock_pi()"
  * Merge "Revert "mmc: sdhci-msm: Add support to vote for vdd io parent regulator""
  * mmc: sdhci-msm: Switch to required pad voltage for vbias bypass
  * mmc: sdhci-msm: Add vbias bypass specific hw WA flag
  * Revert "mmc: sdhci-msm: Add support to vote for vdd io parent regulator"
  * f2fs: correct comment in segment.h
  * f2fs: improve sbi status info in debugfs/f2fs/status
  * f2fs: compress: avoid duplicate counting of valid blocks when read compressed file
  * f2fs: fix to do sanity check for sb/cp fields correctly
  * f2fs: avoid unneeded memory allocation in __add_ino_entry()
  * Merge "clk: qcom: gpucc-khaje: Remove NOM_L1 fmax corner"
  * defconfig: kona: Add CONFIG_AQFWD for RB5 board
  * Merge "defconfig: arm64: msm: Enable CTI_SAVE_DISABLE for SDM660"
  * Merge "usb: gadget: Do not disconnect unregistered dev"
  * clk: qcom: gpucc-khaje: Remove NOM_L1 fmax corner
  * Merge "mmc: sdhci: Avoid dumping registers when SD card removed"
  * mmc: sdhci: Avoid dumping registers when SD card removed
  * FM: Fix for no sound issue in FM Audio
  * usb: gadget: Do not disconnect unregistered dev
  * msm: adsprpc: Lock list before removing node
  * HID: add HID_QUIRK_INCREMENT_USAGE_ON_DUPLICATE for Gamevice devices
  * defconfig: kona: Make configs required to pass CTS tests mandatory
  * UPSTREAM: HID: playstation: Add DualSense player LED support
  * UPSTREAM: HID: playstation: Add microphone mute support for DualSense
  * UPSTREAM: HID: playstation: Add initial DualSense lightbar support
  * UPSTREAM: HID: playstation: Fix unused variable in ps_battery_get_property
  * UPSTREAM: HID: playstation: Fix array size comparison (off-by-one)
  * BACKPORT: HID: playstation: Report DualSense hardware and firmware version
  * UPSTREAM: HID: playstation: Add DualSense accelerometer and gyroscope
  * BACKPORT: HID: playstation: Add DualSense classic rumble support
  * Merge "TREAM: HID: playstation: Track devices in list"
  * Merge "soc: qcom: socinfo: Add support for kona iot soc-id"
  * msm: pcie: Add ASM2806 PCIe device and vendor id
  * TREAM: HID: playstation: Track devices in list
  * UPSTREAM: HID: playstation: Add DualSense touchpad support
  * UPSTREAM: HID: playstation: Add DualSense battery support
  * UPSTREAM: HID: playstation: Add DualSense Bluetooth support
  * UPSTREAM: HID: playstation: Use DualSense MAC address as unique identifier
  * defconfig: arm64: msm: Enable CTI_SAVE_DISABLE for SDM660
  * UPSTREAM: HID: playstation: Initial DualSense USB support
  * f2fs: extent cache: support unaligned extent
  * f2fs: Kconfig: clean up config options about compression
  * Merge "cnss2: Add code to notify mhi about link down"
  * Merge "defconfig: Enable smb1394 driver for khaje"
  * defconfig: Enable smb1394 driver for khaje
  * Merge "Revert "kbuild: force to build vmlinux if CONFIG_MODVERSION=y""
  * Merge "coresight-tmc-etr: remove mem_lock when call usb_qdss_close"
  * Merge "usb: f_qdss: Flush connect_work in qdss close"
  * coresight-tmc-etr: remove mem_lock when call usb_qdss_close
  * Merge "defconfig: arm: msm: Enable debugcc config for SDM439"
  * f2fs: reduce the scope of setting fsck tag when de->name_len is zero
  * f2fs: fix to stop filesystem update once CP failed
  * f2fs: introduce discard_unit mount option
  * f2fs: fix min_seq_blocks can not make sense in some scenes.
  * f2fs: fix to force keeping write barrier for strict fsync mode
  * f2fs: fix wrong checkpoint_changed value in f2fs_remount()
  * f2fs: show sbi status in debugfs/f2fs/status
  * f2fs: turn back remapped address in compressed page endio
  * f2fs: change fiemap way in printing compression chunk
  * f2fs: do not submit NEW_ADDR to read node block
  * f2fs: compress: remove unneeded read when rewrite whole cluster
  * Merge "defconfig: sdm660: Add defconfig changes for camera"
  * Merge "mhi: core: Notify all mhi client's for PCIe link down recovery"
  * Merge "dt-bindings: Add gpr header required for AR on kona"
  * Merge "coresight: tmc: fix spinlock bad magic/usb_qdss_close"
  * soc: qcom: socinfo: Add support for kona iot soc-id
  * defconfig: sdm660: Add defconfig changes for camera
  * usb: f_qdss: Flush connect_work in qdss close
  * coresight: tmc: fix spinlock bad magic/usb_qdss_close
  * scsi: ufs-qcom: fix the issue of get wrong ufs clk status from sysfs node
  * futex: Ensure the correct return value from futex_lock_pi()
  * defconfig: arm: msm: Enable debugcc config for SDM439
  * dt-bindings: Add gpr header required for AR on kona
  * Merge "drivers: thermal: Add support for RPM SMD cooling device"
  * Merge "clk: qcom: gpucc: Update the post div ops for GPUCC PLL for KHAJE"
  * clk: qcom: gpucc: Update the post div ops for GPUCC PLL for KHAJE
  * clk: qcom: clk-alpha-pll: Update implementation for Zonda PLL
  * drivers: thermal: Add support for RPM SMD cooling device
  * diag: Avoid reallocating if buffer supports hdlc max pkt size
  * Merge "diag: Update log and event mask code ranges"
  * Merge 9537a1d5c674db9e81d73fa733b925a9c7124386 on remote branch
  * mhi: core: Fix find_last_bit() usage
  * diag: Update log and event mask code ranges
  * Merge "drivers: rpmh: Always bug_on() upon timeout in rpmh_write_batch()"
  * drivers: rpmh: Always bug_on() upon timeout in rpmh_write_batch()
  * clk: qcom: mdss: Fix flicker issue at early boot of kernel
  * Merge "msm: kgsl: Fix memory leak for anonymous buffers"
  * Merge "drivers: can: mcp25xxfd: Remove return 0 for void function"
  * drivers: can: mcp25xxfd: Remove return 0 for void function
  * f2fs: don't sleep while grabing nat_tree_lock
  * Revert "kbuild: force to build vmlinux if CONFIG_MODVERSION=y"
  * Merge "qxr-stdalonevwr: changes for oracle mic bias"
  * f2fs: remove allow_outplace_dio()
  * f2fs: make f2fs_write_failed() take struct inode
  * Merge "msm: ipa3: Enabling the HOLB on USB consumer pipe for APQ target"
  * msm: ipa3: Enabling the HOLB on USB consumer pipe for APQ target
  * Merge "soc: qcom: dcc: Avoid dcc_sram_writel overflow"
  * Merge "mmc: host: OTA upgrade scenario support for multi fs combination"
  * Merge "defconfig: msm8937_64: Update the defconfigs"
  * soc: qcom: dcc: Avoid dcc_sram_writel overflow
  * Merge "config: enable dcc driver"
  * Merge "soc: qcom: dcc: Avoid huge memory allcation"
  * qxr-stdalonevwr: changes for oracle mic bias
  * soc: qcom: dcc: Avoid huge memory allcation
  * defconfig: msm8937_64: Update the defconfigs
  * soc: qcom: dcc: Use sscanf instead of strlcpy
  * power: qpnp-qg: Fix a DT property name
  * mmc: host: OTA upgrade scenario support for multi fs combination
  * Merge "usb: f_fs: Free descriptors in func_unbind"
  * Merge "usb: gadget: f_rndis: Add 'wceis' flag to indicate 'Wireless' RNDIS"
  * f2fs: quota: fix potential deadlock
  * f2fs: let's keep writing IOs on SBI_NEED_FSCK
  * f2fs: Revert "f2fs: Fix indefinite loop in f2fs_gc() v1"
  * Merge "drivers: cpuidle: lpm-levels: Correct missing list initialize"
  * Merge "pinctrl: Update reserved gpio list for Khaje"
  * Merge "serial: msm_geni_serial: Add Workqueue to dump the registers"
  * Merge "serial: msm_geni_serial: Reset UART error code to default during shutdown"
  * drivers: cpuidle: lpm-levels: Correct missing list initialize
  * serial: msm_geni_serial: Reset UART error code to default during shutdown
  * Merge "From bbfaa7d36c1eb465f120f2a3dfe25c1fe022195d Mon Sep 17 00:00:00 2001 From: KaiChieh Chuang <kaichieh.chuang@mediatek.com> Date: Thu, 7 Mar 2019 07:51:09 +0800 Subject: [PATCH] ASoC: dpcm: prevent snd_soc_dpcm use after free"
  * serial: msm_geni_serial: Add Workqueue to dump the registers
  * Merge "defconfig: Enable power delivery on Khaje"
  * Merge "serial: msm_geni_serial: Bailout from suspend if RX data is pending"
  * Merge "diag: Update log and event mask code ranges"
  * Merge "cnss2: Add code to do PCIe L2 suspend during shutdown"
  * cnss2: Add code to do PCIe L2 suspend during shutdown
  * pinctrl: Update reserved gpio list for Khaje
  * Merge "include: uapi: Add support for 128x32 alignment of NV12"
  * config: enable dcc driver
  * soc: qcom: dcc: Add snapshot of qcom-dcc driver
  * serial: msm_geni_serial: Bailout from suspend if RX data is pending
  * diag: Update log and event mask code ranges
  * Merge "clk: qcom: alpha-pll: Update Alpha PLL width for Zonda PLL"
  * usb: f_fs: Free descriptors in func_unbind
  * clk: qcom: alpha-pll: Update Alpha PLL width for Zonda PLL
  * usb: gadget: f_rndis: Add 'wceis' flag to indicate 'Wireless' RNDIS
  * Merge "defconfig: Enable refgen regulator driver for khaje"
  * Merge "clk: qcom: khaje: Update clock changes for GCC/GPUCC/DISPCC"
  * clk: qcom: khaje: Update clock changes for GCC/GPUCC/DISPCC
  * Merge "msm: ipa3: Fix to prevent Integer Overflow"
  * f2fs: avoid to create an empty string as the extension_list
  * f2fs: compress: fix to set zstd compress level correctly
  * f2fs: add sysfs nodes to get GC info for each GC mode
  * Merge "clk: qcom: cpu: Add support to L2 PC latency for SDM429/39"
  * defconfig: disable per-cgroup pressure tracking
  * BACKPORT: cgroup: make per-cgroup pressure stall tracking configurable
  * Merge "clk: qcom: debugcc: Add cpucc debugcc support for SDM439"
  * Merge "clk: qcom: rcg2: Force enable the RCG before update for GFX"
  * clk: qcom: cpu: Add support to L2 PC latency for SDM429/39
  * clk: qcom: cpu-sdm: Add cpu clock driver for SDM439
  * dt-bindings: clock: Add support for CPUCC clock ids for SDM439
  * clk: qcom: Add LPM Latency support for CPU clocks
  * clk: qcom: clk-pll: Add support for SPM event in PLL
  * Merge "msm: ADSPRPC: Fix deadlock during SSR"
  * Merge "serial: msm_geni_serial: Reduce wait for transfer delay"
  * serial: msm_geni_serial: Reduce wait for transfer delay
  * usb: f_fs: Avoid use-after-free of ffs_data
  * include: uapi: Add support for 128x32 alignment of NV12
  * defconfig: Enable refgen regulator driver for khaje
  * Merge 5d34a1294922540aa83240e425bb48ff72254f49 on remote branch
  * Merge "msm: kgsl: Update GMU FW version for A619 GPU"
  * Merge "power: smblite: Fix max_limit_ma for smblite"
  * clk: qcom: rcg2: Force enable the RCG before update for GFX
  * mhi: core: Notify all mhi client's for PCIe link down recovery
  * power: smblite: Fix max_limit_ma for smblite
  * msm: ipa3: Fix to prevent Integer Overflow
  * Merge "dcc_v2: Control the cti trigger of each link list individually"
  * dcc_v2: Control the cti trigger of each link list individually
  * msm: kgsl: Fix memory leak for anonymous buffers
  * defconfig: Enable remaining defconfigs
  * Merge "defconfig: disable per-cgroup pressure tracking"
  * Merge "defconfig: arm64: msm: Enable MSM_DEBUG_LAR_UNLOCK for msm8937"
  * f2fs: drop dirty node pages when cp is in error status
  * Merge "clk: qcom: gcc: Add GPLL6 fixed factor support for qm215"
  * Merge "clk: qcom: gcc: Correct the frequencies in freq_tbl for SDM439"
  * clk: qcom: debugcc: Add cpucc debugcc support for SDM439
  * clk: qcom: gcc: Correct the frequencies in freq_tbl for SDM439
  * clk: qcom: gcc: Add GPLL6 fixed factor support for qm215
  * Merge "soc: qcom: Return correct error code when program_key fails fails"
  * cnss2: Add code to notify mhi about link down
  * defconfig: disable per-cgroup pressure tracking
  * defconfig: arm64: msm: Enable MSM_DEBUG_LAR_UNLOCK for msm8937
  * defconfig: arm: msm: Enable MSM_DEBUG_LAR_UNLOCK for msm8937
  * soc: mem_dump: Enable MSM_DEBUG_LAR_UNLOCK for MSM8937
  * defconfig: sdm660: Add missing coresight configs
  * From bbfaa7d36c1eb465f120f2a3dfe25c1fe022195d Mon Sep 17 00:00:00 2001 From: KaiChieh Chuang <kaichieh.chuang@mediatek.com> Date: Thu, 7 Mar 2019 07:51:09 +0800 Subject: [PATCH] ASoC: dpcm: prevent snd_soc_dpcm use after free
  * soc: qcom: Return correct error code when program_key fails fails
  * Merge "usb: dwc3: Use pwr_evt_irq to wakeup if dp/dm directly connected to GIC"
  * msm: ADSPRPC: Fix deadlock during SSR
  * soc: qcom: bam_dmux: Skip disconnect call on bootup
  * f2fs: initialize page->private when using for our internal use
  * usb: dwc3: Use pwr_evt_irq to wakeup if dp/dm directly connected to GIC
  * BACKPORT: cgroup: make per-cgroup pressure stall tracking configurable
  * f2fs: compress: add nocompress extensions support
  * usb: dwc3-msm: Enable pwr_evt_irq for wakeup after LPM is done
  * ANDROID: Incremental fs: Set credentials before reading/writing
  * ANDROID: Incremental fs: fix up attempt to copy structures with READ/WRITE_ONCE
  * Merge "Revert "clk: qcom: Add enable_safe_config for gfx3d_clk_src""
  * Merge "coresight-tmc-etr: Fix deadlock issue while switching mode"
  * Merge "Change the subprocess argument to fix the build issue"
  * Merge "clk: qcom: gcc: Add halt check for MSS clocks for SDM660"
  * Merge "clk: qcom: mmcc: Change halt to halt_voted for SDM660"
  * Merge "msm: adsprpc: Handle UAF in process shell memory"
  * Merge "msm: kgsl: Make sure gpu-speed-bin-vectors has the correct size"
  * Merge "msm: kgsl: Keep private intact until last refcount is put"
  * Merge "mmc: cqhci: Handle cqhci crypto configurations correctly"
  * Merge "power: qpnp-qg: Add support to clear soh upon first profile load"
  * power: qpnp-qg: Add support to clear soh upon first profile load
  * Merge "dt-bindings: msm: Add bindings for flash type"
  * Merge "include: uapi: Add QBG UAPI headers"
  * Merge "defconfig: Enable AW2016 LED driver for khaje"
  * msm: kgsl: Make sure gpu-speed-bin-vectors has the correct size
  * msm: kgsl: Add multiple fuses based speed bin
  * Change the subprocess argument to fix the build issue
  * include: uapi: Add QBG UAPI headers
  * defconfig: Enable AW2016 LED driver for khaje
  * Merge 54a42cb3880d7bd921c8a5f84886648d66c71278 on remote branch
  * clk: qcom: gcc: Add halt check for MSS clocks for SDM660
  * msm: adsprpc: Handle UAF in process shell memory
  * msm: camera: Add gpio based flash driver support
  * leds: aw2016: update breath sysfs node name
  * Use environment variable to find unifdef tool
  * kernel_headers: Explicitly run headers_install under 'sh'
  * dt-bindings: msm: Add bindings for flash type
  * clk: qcom: mmcc: Change halt to halt_voted for SDM660
  * Merge "leds: add initial driver for AW2016 LED device"
  * msm: kgsl: Keep private intact until last refcount is put
  * Merge "clk: qcom: khaje: Update freq_tbl and pll configurations"
  * Revert "f2fs: avoid attaching SB_ACTIVE flag during mount/remount"
  * f2fs: remove false alarm on iget failure during GC
  * clk: qcom: khaje: Update freq_tbl and pll configurations
  * defconfig: Enable power delivery on Khaje
  * coresight-tmc-etr: Fix deadlock issue while switching mode
  * defconfig: Enable memory cgroup config
  * leds: add initial driver for AW2016 LED device
  * Merge "pinctrl: Update PINCTRL macro definition for Khaje"
  * pinctrl: Update PINCTRL macro definition for Khaje
  * f2fs: enable extent cache for compression files in read-only
  * f2fs: fix to avoid adding tab before doc section
  * f2fs: introduce f2fs_casefolded_name slab cache
  * f2fs: swap: support migrating swapfile in aligned write mode
  * f2fs: swap: remove dead codes
  * f2fs: compress: add compress_inode to cache compressed blocks
  * f2fs: clean up /sys/fs/f2fs/<disk>/features
  * f2fs: add pin_file in feature list
  * f2fs: Advertise encrypted casefolding in sysfs
  * f2fs: Show casefolding support only when supported
  * f2fs: support RO feature
  * f2fs: logging neatening
  * f2fs: restructure f2fs page.private layout
  * Merge "pinctrl: msm: add range of reserved gpio for Khaje"
  * Merge "input: touchscreen: nt36xxx: Add Chip id and firmware"
  * Merge "ANDROID: Incremental fs: Set credentials before reading/writing"
  * Merge "msm: kgsl: Remove sysfs entries after releasing memory"
  * input: touchscreen: nt36xxx: Add Chip id and firmware
  * Merge "pinctrl: Add wakeup GPIO register and bit information for khaje"
  * Merge "mhi: core: Prevent doorbell with invalid tre"
  * Merge "defconfig: khaje: Enable khaje pinctrl"
  * ANDROID: Incremental fs: Set credentials before reading/writing
  * pinctrl: Add wakeup GPIO register and bit information for khaje
  * defconfig: khaje: Enable khaje pinctrl
  * Merge "defconfig: disable debug configs"
  * Merge "pinctrl: qcom: Add support for Khaje SoC pin control"
  * pinctrl: msm: add range of reserved gpio for Khaje
  * defconfig: disable debug configs
  * ANDROID: Incremental fs: fix up attempt to copy structures with READ/WRITE_ONCE
  * Merge "msm: ipa: Fix array out of bound and use after NULL check"
  * Merge "dt-bindings: msm: Add bindings for flash type"
  * mmc: cqhci: Handle cqhci crypto configurations correctly
  * disp: uapi: drm: Add fsc format modifier
  * Revert "clk: qcom: Add enable_safe_config for gfx3d_clk_src"
  * Merge "msm: adsprpc: Remove bad ioctl log"
  * mhi: core: Prevent doorbell with invalid tre
  * msm: adsprpc: Remove bad ioctl log
  * pinctrl: qcom: Add support for Khaje SoC pin control
  * Merge "usb: dt-bindings: Add USB QMP PHY registers definition for Khaje"
  * dt-bindings: msm: Add bindings for flash type
  * Merge "defconfig: Add HSUSB PHY configurations to Khaje"
  * msm: ipa: Fix array out of bound and use after NULL check
  * msm: ipa: Fix to check only reset IPA stats can have data as NULL
  * msm: ipa: Fix pointer checked for NULL may be used
  * Merge "leds: led-class: Retain the latest user brightness request"
  * Merge "mmc: core: Add at least 3 mclk cycle delay before next command after ACMD41"
  * msm: kgsl: Remove sysfs entries after releasing memory
  * msm: kgsl: Remove debugfs directory inside lock
  * Merge "soc: qcom: Add check to handle out of bound access"
  * Merge "FM: Reduce high scan time"
  * leds: led-class: Retain the latest user brightness request
  * leds: led-class: add support for max_brightness store
  * mmc: core: Add at least 3 mclk cycle delay before next command after ACMD41
  * FM: Reduce high scan time
  * soc: qcom: Add check to handle out of bound access
  * Merge "md: dm-default-key: Override dun value for selected bios"
  * Merge commit '970f4dd431975a2d2771482b4cac83a200927867' into kernel.lnx.4.19.r3
  * defconfig: Enable CONFIG_MSM_TZ_LOG for 8937-perf
  * Merge "HID: make arrays usage and value to be the same"
  * Merge "netfilter: nf_conntrack_h323: lost .data_len definition for Q.931/ipv6"
  * HID: make arrays usage and value to be the same
  * Merge "diag: Update log and event mask code ranges"
  * Merge "defconfig: khaje: Add support for clock controllers"
  * md: dm-default-key: Override dun value for selected bios
  * Merge "diag: Add debug logs while reading masks into userspace"
  * Merge "phy-msm-usb: Add mutex for protecting msm_otg_reset"
  * Merge "clk: qcom: dispcc: Add support for dispcc driver for KHAJE"
  * Merge "md: dm-default-key: Fix IV with set_dun flag defined"
  * Merge "Avoid fatal error if ICE registers are not defined"
  * Merge "cnss2: Log SW_CTRL GPIO value if PCIe link training fails"
  * Merge "fbdev: msm: Avoid race condition while iommu mapping from DSI flow"
  * Merge "camera: smmu: fix for incorrect populated sids"
  * Merge "rpmsg: qcom_smd: Return error if receive callback is not present"
  * defconfig: khaje: Add support for clock controllers
  * clk: qcom: debugcc: Add support for Debugcc for KHAJE
  * fbdev: msm: Avoid race condition while iommu mapping from DSI flow
  * md: dm-default-key: Fix IV with set_dun flag defined
  * cnss2: Log SW_CTRL GPIO value if PCIe link training fails
  * diag: Update log and event mask code ranges
  * Merge "msm: kgsl: Fix nr_removed calculation while reducing pools"
  * Merge "clk: qcom: gpucc: Add Graphics Clock controller for KHAJE"
  * Merge "msm: npu: Use spinlock for ipc write to avoid context switch"
  * Merge "msm: ipa3: Fix to change the tag process timeout value in cleanup"
  * Merge "soc: qcom: pil: Reuse carveout region for mdt header"
  * camera: smmu: fix for incorrect populated sids
  * msm: npu: Use spinlock for ipc write to avoid context switch
  * msm: kgsl: Fix nr_removed calculation while reducing pools
  * Avoid fatal error if ICE registers are not defined
  * rpmsg: qcom_smd: Return error if receive callback is not present
  * clk: qcom: dispcc: Add support for dispcc driver for KHAJE
  * Merge "serial: msm_geni_serial: Add ioctl for sending uart error codes to BT host"
  * Merge "msm: kgsl: Flush mem workqueue and retry if failed to find SVM region"
  * phy-msm-usb: Add mutex for protecting msm_otg_reset
  * Merge "wil6210: Drop unicast sub frame if part of a multicast amsdu"
  * Merge "dt-bindings: clock: gcc-khaje: Add support for USB3 MUX and BCR"
  * wil6210: Drop unicast sub frame if part of a multicast amsdu
  * clk: qcom: gpucc: Add Graphics Clock controller for KHAJE
  * clk: qcom: gcc-khaje: Add GCC support for KHAJE
  * Merge "fbdev: msm: Increment commit count during null commit"
  * dt-bindings: clock: gcc-khaje: Add support for USB3 MUX and BCR
  * msm: ipa3: Fix to change the tag process timeout value in cleanup
  * usb: dt-bindings: Add USB QMP PHY registers definition for Khaje
  * power: supply: smb1398-charger: disable WIN_OV deglitch for SMB1394
  * serial: msm_geni_serial: Add ioctl for sending uart error codes to BT host
  * Merge "clk: qcom: Add enable_safe_config for gfx3d_clk_src"
  * Merge "mmc: cmdq_hci: Notify sdhci for enhanced strobe"
  * f2fs: introduce FI_COMPRESS_RELEASED instead of using IMMUTABLE bit
  * f2fs: compress: remove unneeded preallocation
  * f2fs: avoid attaching SB_ACTIVE flag during mount/remount
  * f2fs: atgc: export entries for better tunability via sysfs
  * f2fs: compress: fix to disallow temp extension
  * f2fs: let's allow compression for mmap files
  * f2fs: add MODULE_SOFTDEP to ensure crc32 is included in the initramfs
  * f2fs: return success if there is no work to do
  * defconfig: Enable SDM429 configs
  * mmc: cmdq_hci: Notify sdhci for enhanced strobe
  * mmc: sdhci-msm: Skip PWRSAVE_DLL setting for sdcc minor verion 0x4D
  * clk: qcom: Add enable_safe_config for gfx3d_clk_src
  * Merge "msm: camera: QM215 delay during power up sequence"
  * fbdev: msm: Increment commit count during null commit
  * msm: mdss: clear fences in ESD panel dead scenario
  * defconfig: Add HSUSB PHY configurations to Khaje
  * wil6210: check integrity of received AMSDU packets
  * Merge "wil6210: Drop plaintext frames on secure network" into kernel.lnx.4.19.r3
  * Merge "wil6210: AP should not forward eapol packets" into kernel.lnx.4.19.r3
  * wil6210: Drop plaintext frames on secure network
  * wil6210: AP should not forward eapol packets
  * msm: ipa3: Fix to copy num of rules from user space
  * Merge "power: smb1398: Add support for SMB1394"
  * defconfig: Enable SDM429 configs
  * diag: Add debug logs while reading masks into userspace
  * Merge "defconfig: Enable config CONFIG_UTS_NS for msm8937_32"
  * Merge "tty: serial: msm: Add suspend resume support"
  * Merge "wil6210: check integrity of received AMSDU packets"
  * f2fs: compress: clean up parameter of __f2fs_cluster_blocks()
  * f2fs: compress: remove unneeded f2fs_put_dnode()
  * f2fs: atgc: fix to set default age threshold
  * f2fs: Prevent swap file in LFS mode
  * f2fs: fix to avoid racing on fsync_entry_slab by multi filesystem instances
  * f2fs: add cp_error check in f2fs_write_compressed_pages
  * f2fs: compress: rename __cluster_may_compress
  * tty: serial: msm: Add suspend resume support
  * Merge "msm: camera: Add reset logic for snps phy"
  * defconfig: Enable config CONFIG_UTS_NS for msm8937_32
  * Merge "usb: f_qdss: Avoid wait_for_completion if qdss_disable is called"
  * Merge "msm: drm: uapi: add rounded corner uapi"
  * Merge "f_uac2: increase number of requests to 32 for UAC2"
  * Merge "msm: ipa: Fix considering prefetch buf size when mapping"
  * Merge "sysfs: ufs-qcom: Add sysfs entries for flashpvl"
  * Merge "defconfig: Enable SDM429 configs"
  * Merge "USB: dwc3: gadget: Queue data for 16 micro frames ahead in future"
  * Merge "clk: qcom: mdss: fix blank / un-blank issue for DSI 12nm pll"
  * f_uac2: increase number of requests to 32 for UAC2
  * Merge 65e986ff6cf04e39f7c828df3baf2140375c9a91 on remote branch
  * defconfig: Enable SDM429 configs
  * Merge "ipa: Remove overwrite copy"
  * Merge "dt-bindings: clock: Add support for KHAJE clock ids"
  * Merge "usb_bam: Set default BAM type as DWC3 if not specified"
  * Merge "ARM: defconfig: Enable config for msm8937_32go"
  * USB: dwc3: gadget: Queue data for 16 micro frames ahead in future
  * USB: uvc: Fix video quality issues for streaming video over USB
  * USB: dwc3: gadget: Increase TX fifo size for isochronous endpoint
  * power: smb1398: Add support for SMB1394
  * clk: qcom: mdss: fix blank / un-blank issue for DSI 12nm pll
  * msm: camera: Add reset logic for snps phy
  * Merge "vidc_3x: Change to avoid unloading firmware"
  * Merge "defconfig: Enable config for msm8937_32"
  * Merge "defconfig: Enable config for msm8937_64"
  * msm: ipa: Fix considering prefetch buf size when mapping
  * Merge "vidc_3x: Use static table governor for bus voting"
  * usb_bam: Set default BAM type as DWC3 if not specified
  * dt-bindings: clock: Add support for KHAJE clock ids
  * msm: drm: uapi: add rounded corner uapi
  * USB: gadget: f_uvc: Enable required controls for CT and PU
  * vidc_3x: Change to avoid unloading firmware
  * msm: camera: Update SDM439 csiphy driver
  * ipa: Remove overwrite copy
  * Add support for new comanche 1.3 soc ID
  * Merge "max31760: add support for additional registers"
  * Merge "msm: kgsl: Fix snapshot collection for ib1"
  * wil6210: check integrity of received AMSDU packets
  * wil6210: Drop plaintext frames on secure network
  * wil6210: AP should not forward eapol packets
  * Merge "defconfig: Enable config CONFIG_DEBUG_FS for msm8937_64"
  * Merge "msm: ipa3: Fix to copy num of rules from user space"
  * netfilter: nf_conntrack_h323: lost .data_len definition for Q.931/ipv6
  * vidc_3x: Use static table governor for bus voting
  * usb: f_qdss: Avoid wait_for_completion if qdss_disable is called
  * Merge " msm: ipa2: free the skb"
  * Merge "clk: qcom: clk-alpha-pll: Add support for FSM legacy mode"
  * Merge "defconfig: Enable ARCH_KHAJE"
  * Merge "soc: qcom: socinfo: Add soc information for khaje"
  * Merge "defconfig: remove CONFIG_SECURITY_PERF_EVENTS_RESTRICT"
  *  msm: ipa2: free the skb
  * defconfig: Enable ARCH_KHAJE
  * msm: Add initial support for Khaje in Kconfig platform
  * soc: qcom: socinfo: Add soc information for khaje
  * defconfig: Enable config CONFIG_DEBUG_FS for msm8937_64
  * clk: qcom: clk-alpha-pll: Add support for FSM legacy mode
  * sysfs: ufs-qcom: Add sysfs entries for flashpvl
  * scsi: ufs-qcom: Add one vendor specific sysfs group
  * Merge "rpmsg: qcom_smd: Ensure ordering of channel info updates"
  * Merge "msm: camera: Fix for HFR120 crash while"
  * msm: ipa3: Fix to copy num of rules from user space
  * Merge "Revert "ANDROID: security,perf: Allow further restriction of perf_event_open""
  * defconfig: remove CONFIG_SECURITY_PERF_EVENTS_RESTRICT
  * Merge "msm: kgsl: Avoid flooding kernel log with invalid ioctl errors"
  * f2fs: return EINVAL for hole cases in swap file
  * Merge "input: qti-haptics: ensure valid pointer when calling kfree"
  * msm: kgsl: Fix snapshot collection for ib1
  * f2fs: avoid swapon failure by giving a warning first
  * f2fs: compress: fix to assign cc.cluster_idx correctly
  * f2fs: compress: fix race condition of overwrite vs truncate
  * f2fs: compress: fix to free compress page correctly
  * f2fs: support iflag change given the mask
  * f2fs: avoid null pointer access when handling IPU error
  * input: qti-haptics: ensure valid pointer when calling kfree
  * Merge "rpmsg: qcom_smd: Add check for remote state in send api"
  * Merge "msm: camera: Update csiphy data rate"
  * Merge "icnss2: Add support for MHI state info"
  * Merge "soc: qcom: bam_dmux: Abort open/close cmd during SSR"
  * msm: kgsl: Avoid flooding kernel log with invalid ioctl errors
  * rpmsg: qcom_smd: Add check for remote state in send api
  * msm: kgsl: Flush mem workqueue and retry if failed to find SVM region
  * msm: camera: Update csiphy data rate
  * Merge "tty: Fix ->pgrp locking in tiocspgrp()"
  * Merge "soc: qcom: ssr: Enable the irqs before powering up subsystems"
  * icnss2: Add support for MHI state info
  * Merge "icnss2: Add new ipc context to log smp2p related logs"
  * Merge "diag: Update log and msg mask code ranges"
  * Merge "defconfig: arm: Enable CONFIG_HID_NINTENDO for QM215"
  * soc: qcom: ssr: Enable the irqs before powering up subsystems
  * Merge "diag: Copy length and buffer locally to send to userspace client"
  * Merge "fbdev: msm: Handle rotator init failure correctly"
  * Merge "usb: gadget: f_cdev: Fix use after free of port in f_cdev"
  * icnss2: Add new ipc context to log smp2p related logs
  * diag: Update log and msg mask code ranges
  * ARM: defconfig: Enable config for msm8937_32go
  * usb: gadget: f_cdev: Fix use after free of port in f_cdev
  * Merge "defconfig: arm64: Enable CONFIG_HID_NINTENDO for QM215"
  * defconfig: arm64: Enable CONFIG_HID_NINTENDO for QM215
  * defconfig: arm: Enable CONFIG_HID_NINTENDO for QM215
  * Merge "clk: qcom: gcc: Change rate max values for SDM439"
  * defconfig: Enable config for msm8937_32
  * Merge "USB: composite: Increase the IN endpoint buffer allocation"
  * fbdev: msm: Handle rotator init failure correctly
  * Merge "leds: qpnp-flash-v2: Add support to control flash ramp time"
  * tty: Fix ->pgrp locking in tiocspgrp()
  * max31760: add support for additional registers
  * Merge "ARM: decompressor: avoid speculative prefetch from protected regions"
  * Revert "ANDROID: security,perf: Allow further restriction of perf_event_open"
  * USB: composite: Increase the IN endpoint buffer allocation
  * USB: f_fs: Avoid NULL pointer dereference during epfile_io()
  * USB: gadget: f_fs: Allocate extra buffer for IN endpoint
  * USB: f_serial/f_acm: Increase the IN endpoint buffer allocation
  * USB: f_fs: Increase the IN endpoint buffer allocation
  * leds: qpnp-flash-v2: Add support to control flash ramp time
  * Merge "msm: vidc_3x: correct the flags set for bus mode"
  * Merge "defconfig: msm8937_64: Disable BUILD_ARM64_APPENDED_DTB_IMAGE flag"
  * Merge "defconfig: msm8937_32: Disable BUILD_ARM64_APPENDED_DTB_IMAGE flag"
  * Merge "msm: vidc_3x: Correct the arguments type to align with format string"
  * msm: vidc_3x: Correct the arguments type to align with format string
  * msm: vidc_3x: correct the flags set for bus mode
  * Merge "diag: Use valid data_source for a valid token"
  * Merge "diag: Prevent out of bound write while sending dci pkt to remote"
  * defconfig: msm8937_32: Disable BUILD_ARM64_APPENDED_DTB_IMAGE flag
  * defconfig: msm8937_64: Disable BUILD_ARM64_APPENDED_DTB_IMAGE flag
  * soc: qcom: pil: Reuse carveout region for mdt header
  * clk: qcom: gcc: Change rate max values for SDM439
  * defconfig: msm8937_32go: Disable BUILD_ARM64_APPENDED_DTB_IMAGE flag
  * Merge "qseecom: Update correct parameters before sending to smcinvoke"
  * f2fs: drop inplace IO if fs status is abnormal
  * f2fs: compress: remove unneed check condition
  * f2fs: clean up left deprecated IO trace codes
  * f2fs: avoid using native allocate_segment_by_default()
  * f2fs: remove unnecessary struct declaration
  * qseecom: Update correct parameters before sending to smcinvoke
  * Merge "msm: kgsl: Add A504 GPU support for SDM429"
  * serial: msm_geni_serial: Bypass Flow control lines from termios
  * Merge "i3c: i3c-master-qcom-geni: Force the xfer mode as DMA mode"
  * Merge 42c4658522229d27921659ab615e0f2115d5a95e on remote branch
  * msm: ADSPRPC: Add extra checks for Unsigned request
  * Merge "defconfig: Enable default-key driver for msm8937_64"
  * Merge "defconfig: Enable default-key driver for msm8937_32"
  * Merge "config: enable rmnet_data, msm_rmnet_bam driver"
  * Merge "msm: mdss: Fix race condition while iommu mapping from DSI flow"
  * msm: camera: QM215 delay during power up sequence
  * Merge "defconfig: Enable MSM8937 and SDM439 config"
  * Merge "cnss2: Add CNSS_BUS_EVENT to report bus info"
  * Merge "regulator: qpnp-lcdb: Add n_voltages property for LCDB regulators"
  * Merge "drivers: rpmsg: fix to avoid dump stack in rpm-smd driver"
  * Merge "spi: spi-geni-qcom: Set IOEB for MHI on SPI RX only case"
  * Merge "defconfig: Add Radio_IRIS related config parameter"
  * Merge "defconfig: Enable SDM439 config"
  * config: enable rmnet_data, msm_rmnet_bam driver
  * msm: kgsl: Add A504 GPU support for SDM429
  * defconfig: Enable MSM8937 and SDM439 config
  * defconfig: Enable SDM439 config
  * defconfig: Enable MSM8937 and SDM439 config
  * defconfig: Enable default-key driver for msm8937_32
  * defconfig: Enable default-key driver for msm8937_64
  * defconfig: Add Radio_IRIS related config parameter
  * defconfig: Add Radio_IRIS related config parameter
  * cnss2: Add CNSS_BUS_EVENT to report bus info
  * cnss2: Add update_uevent API to notify CNSS event
  * Merge "mhi: dev: netdev: Set the mac header to 0 for skbs"
  * Merge "defconfig: msm: Enable dm-snapshot for msm8937_64"
  * Merge "defconfig: msm: Enable dm-snapshot for msm8937_32"
  * Merge "defconfig: msm: Enable dm-snapshot for msm8937_32go"
  * Merge "Add support for new HSP version"
  * Merge "diag: Update log mask code ranges"
  * Merge "defconfig: Enable WCNSS configs"
  * Merge "mmc: sdio: Retune sdio after resume"
  * rpmsg: qcom_smd: Ensure ordering of channel info updates
  * defconfig: Enable WCNSS configs
  * soc: qcom: bam_dmux: Abort open/close cmd during SSR
  * diag: Use valid data_source for a valid token
  * Add support for new HSP version
  * mmc: sdio: Retune sdio after resume
  * diag: Copy length and buffer locally to send to userspace client
  * diag: Update log mask code ranges
  * Merge "config: enable new rmnet_data driver"
  * Merge "config: enable new rmnet_data driver"
  * config: enable new rmnet_data driver
  * regulator: qpnp-lcdb: Add n_voltages property for LCDB regulators
  * Merge "msm: kgsl: Enable content protection for A505 GPU"
  * Merge "iommu/arm-smmu: add option to enable halt/resume of SMMU"
  * Merge "usb: gadget: Add snapshot of USB RMNET Function driver"
  * Merge "tcp: adjust rto_base in retransmits_timed_out()"
  * spi: spi-geni-qcom: Set IOEB for MHI on SPI RX only case
  * diag: Prevent out of bound write while sending dci pkt to remote
  * drivers: rpmsg: fix to avoid dump stack in rpm-smd driver
  * msm: kgsl: Enable content protection for A505 GPU
  * serial: msm_geni_serial: Bypass Flow control lines from termios
  * defconfig: Enable config for msm8937_64
  * f2fs: fix to avoid NULL pointer dereference
  * f2fs: avoid duplicated codes for cleanup
  * f2fs: document: add description about compressed space handling
  * f2fs: clean up build warnings
  * f2fs: fix the periodic wakeups of discard thread
  * f2fs: fix to avoid accessing invalid fio in f2fs_allocate_data_block()
  * f2fs: fix to avoid GC/mmap race with f2fs_truncate()
  * f2fs: set checkpoint_merge by default
  * f2fs: Fix a hungtask problem in atomic write
  * f2fs: fix to restrict mount condition on readonly block device
  * f2fs: introduce gc_merge mount option
  * f2fs: fix to cover __allocate_new_section() with curseg_lock
  * f2fs: fix wrong alloc_type in f2fs_do_replace_block
  * f2fs: delete empty compress.h
  * f2fs: fix a typo in inode.c
  * f2fs: allow to change discard policy based on cached discard cmds
  * f2fs: fix to avoid touching checkpointed data in get_victim()
  * f2fs: fix to update last i_size if fallocate partially succeeds
  * f2fs: fix error path of f2fs_remount()
  * f2fs: fix wrong comment of nat_tree_lock
  * f2fs: fix to avoid out-of-bounds memory access
  * f2fs: don't start checkpoint thread in readonly mountpoint
  * f2fs: do not use AT_SSR mode in FG_GC & high urgent BG_GC
  * f2fs: add sysfs nodes to get runtime compression stat
  * f2fs: fix to use per-inode maxbytes in f2fs_fiemap
  * f2fs: fix to align to section for fallocate() on pinned file
  * f2fs: expose # of overprivision segments
  * f2fs: fix error handling in f2fs_end_enable_verity()
  * f2fs: fix a redundant call to f2fs_balance_fs if an error occurs
  * f2fs: remove unused file_clear_encrypt()
  * f2fs: check if swapfile is section-alligned
  * f2fs: fix last_lblock check in check_swap_activate_fast
  * f2fs: remove unnecessary IS_SWAPFILE check
  * f2fs: Replace one-element array with flexible-array member
  * f2fs: compress: Allow modular (de)compression algorithms
  * f2fs: check discard command number before traversing discard pending list
  * f2fs: update comments for explicit memory barrier
  * f2fs: remove unused FORCE_FG_GC macro
  * f2fs: avoid unused f2fs_show_compress_options()
  * f2fs: fix panic during f2fs_resize_fs()
  * f2fs: fix to allow migrating fully valid segment
  * f2fs: fix a spelling error
  * f2fs: fix a spacing coding style
  * fs: Enable bmap() function to properly return errors
  * f2fs: remove obsolete f2fs.txt
  * iommu/arm-smmu: add option to enable halt/resume of SMMU
  * config: enable new rmnet_data driver
  * usb: gadget: Add snapshot of USB RMNET Function driver
  * Merge "clk: qcom: gcc: Add GCC driver node support for SDM429"
  * Merge "clk: qcom: smd-rpm: Add RPM-SMD clock support for SDM429"
  * Merge "extcon: Initialize blocking notifier while registering"
  * Merge "icnss: Avoid qmi register/unregister in case of qmi failure"
  * extcon: Initialize blocking notifier while registering
  * clk: qcom: gcc: Add GCC driver node support for SDM429
  * Merge "icnss2: Avoid qmi register/unregister in case of qmi failure"
  * icnss: Avoid qmi register/unregister in case of qmi failure
  * icnss2: Avoid qmi register/unregister in case of qmi failure
  * Merge "iommu/arm-smmu: Allocate non-coherent memory for secure pagetables"
  * Merge "cnss2: Add sysfs support for configuring qtimer sync interval"
  * tcp: adjust rto_base in retransmits_timed_out()
  * tcp: retry more conservatively on local congestion
  * tcp: create a helper to model exponential backoff
  * tcp: always set retrans_stamp on recovery
  * tcp: address problems caused by EDT misshaps
  * cnss2: Add sysfs support for configuring qtimer sync interval
  * iommu/arm-smmu: Allocate non-coherent memory for secure pagetables
  * defconfig: msm: Enable dm-snapshot for msm8937_32go
  * defconfig: msm: Enable dm-snapshot for msm8937_32
  * defconfig: msm: Enable dm-snapshot for msm8937_64
  * clk: qcom: clk-pll: Round off req_rate in determine rate
  * Merge "include: uapi: Add charger specific headers for QM215"
  * Merge "cnss2: Check for BT Enable GPIO in QCA6390"
  * include: uapi: Add charger specific headers for QM215
  * Merge "defconfig: Enable USB BAM and SMD configs"
  * Merge "msm: adsprpc: overflow vulnerability by race condition in adsprpc driver"
  * clk: qcom: smd-rpm: Add RPM-SMD clock support for SDM429
  * Merge "ice: add return value for functions where ICE is disabled"
  * Merge "msm: ipa: fix potential race condition ioctls"
  * ice: add return value for functions where ICE is disabled
  * Set the default slot for Full Disk Encryption key to 31
  * defconfig: Enable USB BAM and SMD configs
  * Merge "defconfig: msm: enable thermal efuse driver for kona iot"
  * mhi: dev: netdev: Set the mac header to 0 for skbs
  * Merge "usb: gadget: Setup DMA ops for OTG's gadget devices"
  * Merge "clk: qcom: mdss: update dsi 12nm clock driver"
  * Merge "cnss2: Add support to populate device memory information"
  * msm: adsprpc: overflow vulnerability by race condition in adsprpc driver
  * Merge "defconfig: Enable MDSS PLL for msm8937"
  * Merge "signal: Avoid corrupting si_pid and si_uid in do_notify_parent"
  * Merge "defconfig: Initial defconfig files for MSM8937_64"
  * Merge "defconfig: Enable default-key driver for msm8937_32go"
  * defconfig: Enable USB BAM and SMD configs
  * defconfig: Enable default-key driver for msm8937_32go
  * signal: Avoid corrupting si_pid and si_uid in do_notify_parent
  * defconfig: Enable MDSS PLL for msm8937
  * defconfig: Initial defconfig files for MSM8937_64
  * Merge "ice: add return value for functions where ICE is disabled"
  * ice: add return value for functions where ICE is disabled
  * cnss2: Check for BT Enable GPIO in QCA6390
  * soc: qcom: Enable  MSM_TZ_SMMU msm8937
  * usb: gadget: Setup DMA ops for OTG's gadget devices
  * Merge "fbdev: msm: Enable mdss rotator driver"
  * Merge "cnss2: Add CNSS wrapper function for msm_pcie_reg_dump()"
  * Merge "cnss2: Use unified API to get RDDM and recovery timeouts"
  * fbdev: msm: Enable mdss rotator driver
  * Merge "defconfig: Enable MDSS PLL config for msm8937_32"
  * defconfig: Enable MDSS PLL config for msm8937_32
  * Merge "defconfig: Add Radio_IRIS related config parameter"
  * Merge "clk: qcom: gcc: Add gcc-mdss driver node support for QM215"
  * defconfig: Add Radio_IRIS related config parameter
  * Merge "Don't use gpio_free() for output gpios"
  * cnss2: Use unified API to get RDDM and recovery timeouts
  * cnss2: Add new API to get QMI related timeouts
  * cnss2: Add support to populate device memory information
  * Merge 8a7341818eaed149f14d232da19a0590469554a9 on remote branch
  * Merge "iommu: Update function for ARM_MSM_SECURE io pagetable type"
  * Merge "RPMSG driver changes for FM in msm-4.19"
  * iommu: Update function for ARM_MSM_SECURE io pagetable type
  * Don't use gpio_free() for output gpios
  * Merge "defconfig: Initial defconfig files for MSM8937_32"
  * clk: qcom: gcc: Add gcc-mdss driver node support for QM215
  * Merge "clk: qcom: mdss: Update VCO Clk enums for mdss-dsi-28lpm"
  * RPMSG driver changes for FM in msm-4.19
  * Merge "defconfig: Enable USB BAM configs"
  * Merge "Set the default slot for Full Disk Encryption key to 31"
  * Merge "clk: qcom: debugcc: Add cpucc debugcc support for QM215"
  * Merge "clk: qcom: gcc: Add GPLL3 derived frequencies for qm215"
  * Merge "iio: spmi-vadc: Add option to specify scale-fn-type in devicetree"
  * Merge "cfg80211: Adjust 6 GHz frequency to channel conversion"
  * defconfig: Enable USB BAM configs
  * defconfig: Initial defconfig files for MSM8937_32
  * cnss2: Add CNSS wrapper function for msm_pcie_reg_dump()
  * clk: qcom: gcc: Add GPLL3 derived frequencies for qm215
  * Merge "soc: qcom: smp2p: Add proper retrigger detection"
  * iio: spmi-vadc: Add option to specify scale-fn-type in devicetree
  * iio: adc: spmi-vadc: Add adc support for SPF targets
  * Merge "msm: adsprpc: Clean DMA handles maps in case of  error"
  * Merge "diag: Prevent possible out of bound copy to userspace"
  * Merge "iommu: Do not set COHERENT flag for secure pagetable allocation"
  * cfg80211: Adjust 6 GHz frequency to channel conversion
  * Merge "drivers: soc: qcom: Fix compilation errors"
  * Merge "scsi: ufs: Fix IOCTL error checking for input buffer"
  * iommu: Do not set COHERENT flag for secure pagetable allocation
  * soc: qcom: smp2p: Add proper retrigger detection
  * drivers: soc: qcom: Fix compilation errors
  * Merge "net: ethernet: Add snapshot of RMNET BAM driver"
  * Merge "clk: qcom: mdss: fix blank / unblank for DSI 28nm pll"
  * clk: qcom: mdss: Update VCO Clk enums for mdss-dsi-28lpm
  * dt-bindings: clock: Add mdss-28nm-pll-clk for legacy targets
  * drivers: soc: qcom: Add bam dmux smem state handling
  * Merge "qcom-geni-se: Avoid dumping of GENI registers in console"
  * net: ethernet: Add snapshot of RMNET BAM driver
  * Merge "iommu/arm-smmu: Do not write to slave side protected context banks"
  * Merge "spmi: spmi-pmic-arb-debug: replace ioremap_resource with ioremap"
  * Merge "soc: qcom: Add support for MSM_TZ_SMMU"
  * Merge "msm: mdss: update the power down sequence for DSI 12nm PHY"
  * Merge "msm: mdss: update the DSI 12nm PHY programming sequence"
  * Merge "dm verity: skip verity work on I/O errors when system is shutting down"
  * Merge "net/sched: fix race between deactivation and dequeue for NOLOCK qdisc"
  * Merge "defconfig: enable PSI config for MSM8937_32go"
  * scsi: ufs: Fix IOCTL error checking for input buffer
  * clk: qcom: mdss: fix blank / unblank for DSI 28nm pll
  * soc: qcom: Add support for MSM_TZ_SMMU
  * iommu/arm-smmu: Do not write to slave side protected context banks
  * net/sched: fix race between deactivation and dequeue for NOLOCK qdisc
  * uio: msm_sharedmem: By pass failure for hyp_assign_phys
  * defconfig: enable PSI config for MSM8937_32go
  * spmi: spmi-pmic-arb-debug: replace ioremap_resource with ioremap
  * qcom-geni-se: Avoid dumping of GENI registers in console
  * dm verity: skip verity work on I/O errors when system is shutting down
  * Return Immediately if failed to set to reset gpio state
  * Merge "power: smb5-lib: Fix race conditions for typec power role"
  * msm: adsprpc: Clean DMA handles maps in case of  error
  * diag: Prevent possible out of bound copy to userspace
  * ARM: decompressor: avoid speculative prefetch from protected regions
  * power: smb5-lib: Fix race conditions for typec power role
  * Merge "defconfig: Add WCNSS related config parameter"
  * msm: mdss: update the DSI 12nm PHY programming sequence
  * msm: mdss: update the power down sequence for DSI 12nm PHY
  * msm: ice: Fix stack-out-of-bound erros on kasan builds
  * clk: qcom: debugcc: Add cpucc debugcc support for QM215
  * smcinvoke: Update the correct parameters and result from qseecom
  * msm: ipa: Fix use-after-free in ipa3_alloc_counter_id
  * Merge "dt-bindings: iio: Add scale function for QM215 batt therm"
  * Merge "msm: ipa: correct the pointer in idr for FnR stats counter"
  * Merge "fbdev: msm: update event callback function to perform ESD check"
  * msm: ipa: correct the pointer in idr for FnR stats counter
  * Merge "ANDROID: xt_qtaguid: Remove tag_entry from process list on untag"
  * fbdev: msm: update event callback function to perform ESD check
  * Merge "radio: iris: Add snapshot of iris FM radio support"
  * Merge "msm: ipa3: Fix to null pointer access"
  * ANDROID: xt_qtaguid: Remove tag_entry from process list on untag
  * clk: qcom: mdss: update dsi 12nm clock driver
  * Merge "msm: ipa: fix race condition on PM vote on sys pipes"
  * msm: ipa: fix race condition on PM vote on sys pipes
  * msm: ipa3: Fix to null pointer access
  * Merge "wcnss: Update pm wake api's for 4.19 kernel"
  * Merge "defconfig: Add clock controller support for QM215"
  * radio: iris: Add snapshot of iris FM radio support
  * Merge "qseecom: Add flush_work based on flag"
  * Merge "icnss2: Add handler for SMMU faults"
  * Merge "arm-smmu: add bitmap for secure context banks"
  * Merge "diag: Update event and log mask code ranges"
  * Merge "qtee_shmbridge: update copyright"
  * defconfig: Add WCNSS related config parameter
  * wcnss: Update pm wake api's for 4.19 kernel
  * qtee_shmbridge: update copyright
  * diag: Update event and log mask code ranges
  * arm-smmu: add bitmap for secure context banks
  * iommu/arm-smmu: override writel_relaxed in smmu global address space
  * Merge "platform: msm: Add USB BAM support to ChipIdea/RMNET"
  * qseecom: Add flush_work based on flag
  * Merge "wcnss: Add wcnss snapshot to msm-4.19"
  * Merge changes I3f958739,I927de392 into kernel.lnx.4.19.r3
  * kona_defconfig: Enable hidraw config
  * tty: serial: msm_geni_serial: Resolve race btw stop rx and cancel rx
  * tty: serial: msm_geni_serial: Fix timeout for Rx abort
  * Merge "mm: cma: Print correct request pages"
  * icnss2: Add handler for SMMU faults
  * wcnss: Add wcnss snapshot to msm-4.19
  * Merge "msm: sde: Update rotator OT settings for sdm660"
  * mm: cma: Print correct request pages
  * defconfig: Add clock controller support for QM215
  * clk: qcom: cpu-sdm: Add cpu clock driver for SDM
  * dt-bindings: iio: Add scale function for QM215 batt therm
  * Merge "leds: vibrator: Add snapshot of vibrator driver"
  * Merge "defconfig: Enable chipidea USB controller configs"
  * Merge "msm: kgsl: Access map_count only if entry is successfully allocated"
  * platform: msm: Add USB BAM support to ChipIdea/RMNET
  * Merge "msm: ipa: Enable wdi3 API in ipa3 config"
  * Merge "tty: serial: msm_geni_serial: Resolve race btw stop rx and cancel rx"
  * Merge "rpmsg: qcom_smd: Add NO_SUSPEND flag for smd edge irq"
  * leds: vibrator: Add snapshot of vibrator driver
  * tty: serial: msm_geni_serial: Resolve race btw stop rx and cancel rx
  * msm: kgsl: Access map_count only if entry is successfully allocated
  * tty: serial: msm_geni_serial: Fix timeout for Rx abort
  * defconfig: Enable chipidea USB controller configs
  * msm: sde: Update rotator OT settings for sdm660
  * Merge "msm: mdss: dsi: set backlight handle to NULL in error case"
  * Merge "clk: qcom: mdss: Add check to read the gdsc status"
  * Merge "msm: ipa: Send actual DL flt rule length to Q6"
  * msm: mdss: dsi: set backlight handle to NULL in error case
  * clk: qcom: mdss: Add check to read the gdsc status
  * Merge "msm: npu: Don't send interrupt if command has been consumed"
  * msm: npu: Don't send interrupt if command has been consumed
  * Merge "clk: qcom: clk-pll: Add support for HF PLL Ops"
  * Merge "rtc: rtc-pm8xxx: add support for PM8916 RTC"
  * msm: ipa: Send actual DL flt rule length to Q6
  * Merge "qseecom: Update the error val check"
  * Merge "qseecom: Check error when allocating coherent buffer"
  * sde: rotator: use shared lock while accessing vbif
  * clk: qcom: clk-pll: Add support for HF PLL Ops
  * clk: qcom: Add enable/disable to clk_regmap_mux_div_ops
  * clk: qcom: Update parent map to use parent_map struct
  * qseecom: Update the error val check
  * ARM: qcom: add board config support for sdm439 and sdm429
  * ARM: qcom: Add board config support for msm8937
  * Set the default slot for Full Disk Encryption key to 31
  * msm: ipa: Enable wdi3 API in ipa3 config
  * Merge "qdss_bridge: fix kfree issue"
  * Merge "iio: ch101: Return -ENODEV when get expander GPIO failed"
  * Merge "usb: gadget: Add snapshot of ChipIdea driver"
  * iio: ch101: Return -ENODEV when get expander GPIO failed
  * Merge "msm: kgsl: Set correct values for SMMU protect register for A3xx"
  * Merge "usb: dwc3: core: Add ipc logs when sg lists are used"
  * Merge "usb: dwc3: gadget: Prevent double free scenario for cancelled_list"
  * USB: EHCI: Add MSM Host Controller driver
  * usb: gadget: Add snapshot of ChipIdea driver
  * usb: phy: Add snapshot of PHY msm usb driver
  * qdss_bridge: fix kfree issue
  * rpmsg: glink: do not break from interrupt handler
  * Merge "clk: qcom: clk-debug: Add support to dump GDSC registers"
  * msm: kgsl: Set correct values for SMMU protect register for A3xx
  * Merge "msm: gsi: Using kzalloc instead of devm_kzalloc"
  * Merge "iio: qcom-rradc: Update logic to monitor health of RRADC peripheral"
  * clk: qcom: clk-debug: Add support to dump GDSC registers
  * fs-verity: support reading signature with ioctl
  * fs-verity: support reading descriptor with ioctl
  * fs-verity: support reading Merkle tree with ioctl
  * fs-verity: add FS_IOC_READ_VERITY_METADATA ioctl
  * fs-verity: don't pass whole descriptor to fsverity_verify_signature()
  * fs-verity: factor out fsverity_get_descriptor()
  * f2fs: remove FAULT_ALLOC_BIO
  * f2fs: use blkdev_issue_flush in __submit_flush_wait
  * f2fs: remove a few bd_part checks
  * fs-verity: move structs needed for file signing to UAPI header
  * fs-verity: rename "file measurement" to "file digest"
  * fs-verity: rename fsverity_signed_digest to fsverity_formatted_digest
  * fs-verity: remove filenames from file comments
  * fs-verity: use smp_load_acquire() for ->i_verity_info
  * Merge "diag: Enable graceful transfer of transport"
  * Merge "drivers: char: Fix compilation error for smd pkt driver"
  * usb: dwc3: gadget: Prevent double free scenario for cancelled_list
  * Merge 112597fd2245ed603c0b7785d09f87e4e599036b on remote branch
  * Merge "kona_defconfig: Enable hidraw config"
  * diag: Enable graceful transfer of transport
  * diag: Enable diag over rpmsg communication for adsp
  * drivers: char: Fix compilation error for smd pkt driver
  * defconfig: msm: Enable smd pkt driver for QM215/SDM429/439
  * Merge "dt-bindings: clock: Add support for CPUCC clock ids for QM215"
  * Merge "defconfig: enable MDSS PLL config for MSM8937_32go"
  * Merge "clk: qcom: mdss: update mdss-dsi-28lpm clk names"
  * clk: qcom: mdss: update mdss-dsi-28lpm clk names
  * Merge "smcinvoke: Update the correct parameters and result from qseecom"
  * rtc: rtc-pm8xxx: add support for PM8916 RTC
  * Merge "socinfo: Add socinfo support for sdm450"
  * msm: mdss: Fix race condition while iommu mapping from DSI flow
  * defconfig: enable MDSS PLL config for MSM8937_32go
  * msm: ipa: fix potential race condition ioctls
  * Merge "misc: add qrc ioctl functions"
  * dt-bindings: clock: Add support for CPUCC clock ids for QM215
  * Merge "usb: misc: nb7vpq904m: update suspend and resume function"
  * Merge "Add support for new versions for BT chip"
  * Add support for new versions for BT chip
  * Merge "diag: Sanitize the mempools with pool data size check"
  * smcinvoke: Update the correct parameters and result from qseecom
  * misc: add qrc ioctl functions
  * misc: add qrc_core and qrc_uart
  * Merge "rpmsg: smd: Signal the TX channel that smd driver read data"
  * Merge "coresight: disable the path when enable source link fail"
  * Merge "usb: dwc3: Enable parkmode for Gen1 controllers"
  * Merge "Kconfig: enable default config of cpu freq qcom for msm8953"
  * Merge "clk: qcom: debugcc: Add debugcc support for QM215"
  * diag: Sanitize the mempools with pool data size check
  * Documentation: f2fs: fix typo s/automaic/automatic
  * f2fs: give a warning only for readonly partition
  * socinfo: Add socinfo support for sdm450
  * socinfo: Add socinfo support for msm8953
  * ARM: qcom: Add board config support for sdm450
  * ARM: qcom: Add board config support for msm8953
  * usb: dwc3: Enable parkmode for Gen1 controllers
  * msm: camera: Fix for HFR120 crash while
  * msm: gsi: Using kzalloc instead of devm_kzalloc
  * Merge "msm: pcie: Restore BME for RC after link_down recovery on SBR"
  * clk: qcom: debugcc: Add debugcc support for QM215
  * clk: qcom: gcc: Add support for GCC clock driver
  * Merge "usb: f_diag: Replace ERROR with pr_err"
  * usb: f_diag: Replace ERROR with pr_err
  * Merge "clk: qcom: mdss: add dsi phy 12nm clock"
  * clk: qcom: mdss: add dsi phy 12nm clock
  * dt-bindings: clock: Add 12nm clock device tree bindings
  * rpmsg: qcom_smd: Add NO_SUSPEND flag for smd edge irq
  * rpmsg: qcom_smd: Read data of size equal to fifo size
  * rpmsg: qcom_smd: Read data of size greater than fifo size
  * rpmsg: qcom_smd: Add ipc logging for smd driver
  * Merge "msm: kgsl: Change start variable type to int in kgsl_iommu_add_global"
  * msm: kgsl: Change start variable type to int in kgsl_iommu_add_global
  * Merge "msm: mdss: update MDSS DSI ULPS configuration"
  * Merge "cfg80211: export regulatory_hint_user() API"
  * rpmsg: smd: Signal the TX channel that smd driver read data
  * rpmsg: qcom_smd: Add GET/SET signal support
  * rpmsg: qcom_smd: increase bounce buffer size
  * drivers: rpmsg: Add smd driver module at post core init
  * Merge "qcom: fg-memif: correct timeout condition for memory grant"
  * Merge "serial: msm_geni_serial: Fix false warning for reset failure"
  * kona_defconfig: Enable hidraw config
  * usb: misc: nb7vpq904m: update suspend and resume function
  * iio: qcom-rradc: Update logic to monitor health of RRADC peripheral
  * cfg80211: export regulatory_hint_user() API
  * msm: mdss: update MDSS DSI ULPS configuration
  * fbdev: msm: call pxl clk unprepare during suspend
  * msm: mdss: add support to handle LP_RX_TO/BTA_TO errors for DSI 12nm PHY
  * msm: mdss: perform DSI PHY s/w reset for 12nm PHY during unblank
  * msm: mdss: update the MDSS DSI ULPS exit sequence
  * msm: mdss: add support to program of HSTX drivers for DSI 12nm PHY
  * msm: mdss: update DSI ULPS entry/exit sequence
  * msm: mdss: add support for DSI 12nm PHY in DSI driver
  * Merge "power: smb2: Enable read/writing of Type-C Rp value"
  * Merge "mmc: core: Set cqe_enabled flag correctly"
  * Merge "Revert "cnss2: Dump mhi debug regs on receiving mhi WAKE event cb""
  * Merge "crypto: Fix possible stack out-of-bound error"
  * f2fs: don't grab superblock freeze for flush/ckpt thread
  * f2fs: add ckpt_thread_ioprio sysfs node
  * f2fs: introduce checkpoint_merge mount option
  * f2fs: relocate inline conversion from mmap() to mkwrite()
  * f2fs: fix a wrong condition in __submit_bio
  * defconfig: msm: enable thermal efuse driver for kona iot
  * drivers: thermal: qcom: Add driver to modify thermal zone based on efuse
  * mmc: core: Set cqe_enabled flag correctly
  * serial: msm_geni_serial: Fix false warning for reset failure
  * Kconfig: enable default config of cpu freq qcom for msm8953
  * msm: Add initial support for sdm450 Kconfig platform
  * msm: Add initial support for msm8953 Kconfig platform
  * Revert "cnss2: Dump mhi debug regs on receiving mhi WAKE event cb"
  * Merge "drivers: char: reset signal support for smd channels"
  * Merge "msm: ADSPRPC: Substitute vfs check with flags"
  * msm: pcie: Restore BME for RC after link_down recovery on SBR
  * Merge "icnss: Allow register/unregister driver execution in serial manner"
  * Merge "drivers: irqchip: qcom: Add MSM8953 pin data for MPM"
  * Merge "defconfig: Enable EXT4 configs"
  * msm: ADSPRPC: Substitute vfs check with flags
  * Merge "drivers: iio: enable compile for TDK sensor"
  * Merge "USB: pd: Add support for enabling PD2.0 only as source"
  * Merge "backlight: qcom-wled: Add "qcom,sync-dly" device tree property"
  * Merge "usb: gadget: f_mass_storage: Remove runtime async resume"
  * Merge "usb: dwc3: Issue ENDTRANSFER cmd on ep0 unconditionally"
  * icnss: Allow register/unregister driver execution in serial manner
  * drivers: char: reset signal support for smd channels
  * drivers: char: add stream like read support
  * crypto: Fix possible stack out-of-bound error
  * drivers: irqchip: qcom: Add MSM8953 pin data for MPM
  * defconfig: Enable EXT4 configs
  * drivers: iio: enable compile for TDK sensor
  * power: smb2: Enable read/writing of Type-C Rp value
  * usb: gadget: f_mass_storage: Remove runtime async resume
  * drivers: iio: add Makefile and Kconfig for TDK chirp sensor
  * backlight: qcom-wled: Add "qcom,sync-dly" device tree property
  * Merge "rpm-smd: Remove redundant spinlocks which are not required"
  * usb: dwc3: Issue ENDTRANSFER cmd on ep0 unconditionally
  * coresight: disable the path when enable source link fail
  * Merge "diag: Acquire spinlock for list delete operation"
  * Merge "defconfig: Enable CPR Regulator for msm8937go"
  * qcom: fg-memif: correct timeout condition for memory grant
  * power: qpnp-fg-gen3: Add a property to reset FG BCL device
  * power_supply: Add FG_RESET_CLOCK property
  * Merge "usb: dwc3: gadget: Replace dev with its parent sysdev in ep_disable"
  * Merge "usb: gadget: Clear transfer started flag if endxfer cmd times out"
  * qseecom: Check error when allocating coherent buffer
  * Merge "usb: pd: Add support to disable pps capability"
  * defconfig: Enable CPR Regulator for msm8937go
  * diag: Acquire spinlock for list delete operation
  * f2fs: remove unnecessary initialization in xattr.c
  * f2fs: fix to avoid inconsistent quota data
  * f2fs: flush data when enabling checkpoint back
  * Merge "regulator: qpnp-lcdb: Disable step voltage ramp for PM8150L V3"
  * Merge "drivers: char: msm_smd_pkt: Return error in case driver is not ready"
  * rpm-smd: Remove redundant spinlocks which are not required
  * regulator: qpnp-lcdb: Disable step voltage ramp for PM8150L V3
  * Merge "power: smb5-lib: Query POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN"
  * Merge "fs: crypto: Add support for legacy pfk based FBE"
  * Merge "defconfig: Enable Incremental FS support for msm8937_32go"
  * Merge "clk: qcom: smd-rpm: Add RPM-SMD clock support for QM215"
  * power: smb5-lib: Query POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN
  * defconfig: Enable Incremental FS support for msm8937_32go
  * Merge "msm: kgsl: Show max gpu temperature"
  * Merge "Merge tag 'refs/tags/TDK-Robotics-RB4-0.1-test3'"
  * Merge "media/dvb: Deregister dma buffers fom shmbridge"
  * media/dvb: Deregister dma buffers fom shmbridge
  * Merge tag 'refs/tags/TDK-Robotics-RB4-0.1-test3'
  * msm: kgsl: Show max gpu temperature
  * Merge "backlight: qcom-spmi-wled: Change the SYNC toggle sequence"
  * USB: pd: Add support for enabling PD2.0 only as source
  * usb: gadget: Clear transfer started flag if endxfer cmd times out
  * backlight: qcom-spmi-wled: Change the SYNC toggle sequence
  * Merge "defconfig: Initial defconfig files for MSM8937_32go"
  * USB: pd: Restart host mode in high speed if no usb3 & dp concurrency
  * defconfig: Initial defconfig files for MSM8937_32go
  * Merge "wigig_sensing: treat data ready as deep sleep exit when needed"
  * Merge "bindings: clock: qcom: Add support for clock IDs for QM215"
  * Merge "dt-bindings: clock: Add support for GCC clock ids for SDM429"
  * Merge "rpmsg: qcom_smd: allow smd create device if remote state is closing"
  * Merge "kernel: driver: Added support msm_smd_pkt driver"
  * Merge "diag: Update event and log mask code latest range"
  * TDK-Robotics-RB5 drivers source code files
  * Merge "fastrpc : fastrpc porting for kernel 4.19 SPF targets"
  * diag: Update event and log mask code latest range
  * f2fs: deprecate f2fs_trace_io
  * f2fs: remove unused stat_{inc, dec}_atomic_write
  * f2fs: introduce sb_status sysfs node
  * f2fs: fix to use per-inode maxbytes
  * f2fs: compress: fix potential deadlock
  * libfs: unexport generic_ci_d_compare() and generic_ci_d_hash()
  * f2fs: fix to set/clear I_LINKABLE under i_lock
  * f2fs: fix null page reference in redirty_blocks
  * f2fs: clean up post-read processing
  * f2fs: trival cleanup in move_data_block()
  * f2fs: fix out-of-repair __setattr_copy()
  * f2fs: fix to tag FIEMAP_EXTENT_MERGED in f2fs_fiemap()
  * f2fs: introduce a new per-sb directory in sysfs
  * f2fs: compress: support compress level
  * f2fs: compress: deny setting unsupported compress algorithm
  * clk: qcom: smd-rpm: Add RPM-SMD clock support for QM215
  * fs: crypto: Add support for legacy pfk based FBE
  * fastrpc : fastrpc porting for kernel 4.19 SPF targets
  * qdss_bridge: fix memory leak issue
  * bindings: clock: qcom: Add support for clock IDs for QM215
  * drivers: char: msm_smd_pkt: Return error in case driver is not ready
  * kernel: driver: Added support msm_smd_pkt driver
  * dt-bindings: clock: Add support for GCC clock ids for SDM429
  * Merge pull request #1 from InvenSenseInc/feature/prepare_TDK-Robotics-RB5-0.1-test2
  * Preparation for TDK-Robotics-RB5-0.1-test2
  * Merge "msm: pcie : Extend link recovery for config space access failure"
  * Merge c236ef40e58879bb0678339922b77f25fcced038 on remote branch
  * Merge "coresight: Make coresight functions as inline"
  * wigig_sensing: treat data ready as deep sleep exit when needed
  * usb: dwc3: gadget: Replace dev with its parent sysdev in ep_disable
  * usb: dwc3: gadget: Check controller status with endpoint enable/disable
  * Merge "cnss2: WAR to trigger self-recovery for link recover callback"
  * Merge "pinctrl: Add support for msm8917 pinctrl"
  * coresight: Make coresight functions as inline
  * Merge "net: qrtr: Excessive logging casuing boot failure"
  * Merge "diag: Read from USB without holding spinlock"
  * cnss2: WAR to trigger self-recovery for link recover callback
  * net: qrtr: Excessive logging casuing boot failure
  * pinctrl: Add support for msm8917 pinctrl
  * pinctrl: Add support for msm8937 pinctrl
  * Merge "cnss_prealloc: Add a 128kB to prealloc pool"
  * Merge "ARM: msm: Add board config support for 32 bit bengal apq iot"
  * msm: pcie : Extend link recovery for config space access failure
  * cnss_prealloc: Add a 128kB to prealloc pool
  * Merge "defconfig: set CONFIG_HZ = 250 for Bengal"
  * Merge "power: smb5-lib: Add additional check to exit charge-termination WA"
  * Merge "serial:msm_geni_serial: Handling the cts counter in uart driver"
  * Merge "nl80211: add error messages to nl80211_parse_chandef()"
  * rpmsg: qcom_smd: allow smd create device if remote state is closing
  * Merge "qcom: step-chg-jeita: Add support for jeita fcc scaling"
  * Merge "drivers: irqchip: qcom: Add MSM8937 pin data for MPM"
  * Merge "diag: Handle drops for diag over rpmsg"
  * Merge "msm: pci: print PCIE LTSSM state at DRV suspend and resume"
  * Merge "mhi: core: Check PCIe link status before accessing MHI registers"
  * Merge "cnss2: Increase prealloc table to satisfy latest driver requirement"
  * Merge "cnss2: Dump mhi debug registers on receiving mhi WAKE event cb"
  * Merge "msm:ADSPRPC :Fix to avoid Use after free in fastrpc_internal_munmap"
  * Merge "cnss2: retry mhi suspend in case packets are pending in MHI layer"
  * Merge "qseecom: Add boundary checks between two subsequent fields"
  * Merge "regulator: qpnp-lcdb: fix race between SC interrupt and lcdb enable"
  * Merge "arm64: fix bootloader_memory_limit"
  * Merge "usb: phy: qusb2: Update tune value for host mode"
  * Merge "msm: kgsl: Use pid struct to find the process to reclaim"
  * Merge "serial: msm_geni_serial: Correct start rx sequence"
  * Merge "msm: pcie : Notify client driver on reading FF's"
  * Merge "coresight: cti: Solve boot up issue"
  * Merge "clk: qcom: mdss: Enable mdss 28lpm pll driver"
  * Merge "ARM: qcom: Add board config support for qm215"
  * Merge "regulator: cpr: add snapshot of cpr-regulator driver"
  * First ready to use version (no ref to upstream commit)
  * usb: pd: Add support to disable pps capability
  * nl80211: add error messages to nl80211_parse_chandef()
  * cnss2: Increase prealloc table to satisfy latest driver requirement
  * power: smb5-lib: Add additional check to exit charge-termination WA
  * cnss2: Dump mhi debug registers on receiving mhi WAKE event cb
  * msm:ADSPRPC :Fix to avoid Use after free in fastrpc_internal_munmap
  * qseecom: Add boundary checks between two subsequent fields
  * diag: Read from USB without holding spinlock
  * serial:msm_geni_serial: Handling the cts counter in uart driver
  * arm64: fix bootloader_memory_limit
  * usb: phy: qusb2: Update tune value for host mode
  * Merge "usb: gadget: gsi: Ensure the doorbell is blocked before suspend"
  * Merge "soc: qcom: socinfo: Add support for msm8917 soc-id"
  * coresight: cti: Solve boot up issue
  * cnss2: retry mhi suspend in case packets are pending in MHI layer
  * clk: qcom: mdss: Enable mdss 28lpm pll driver
  * msm: pcie : Notify client driver on reading FF's
  * msm: kgsl: Use pid struct to find the process to reclaim
  * mm: process_reclaim: pass pid struct instead of tgid
  * msm: kgsl: Change vma->vm_file to shmem file
  * qcom: step-chg-jeita: Add support for jeita fcc scaling
  * mm: process_reclaim: skip target_vma
  * mhi: core: Check PCIe link status before accessing MHI registers
  * power: qpnp-smb5: Add DT option to enable JEITA-ARB handling
  * power: smb5-lib: Improve the charge-termination WA handling
  * msm: pci: print PCIE LTSSM state at DRV suspend and resume
  * msm: kgsl: Deregister gpu address on memdesc_sg_virt failure
  * Merge "mm: skip speculative path for non-anonymous COW faults"
  * diag: Handle drops for diag over rpmsg
  * diag: Enable diag over rpmsg communication for modem
  * diag: Enable diag over rpmsg communication for wcnss
  * Merge "net: sch_generic: fix the missing new qdisc assignment"
  * Merge "net: qrtr: print error case for qrtr receive"
  * usb: gadget: gsi: Ensure the doorbell is blocked before suspend
  * msm: kgsl: Correct the refcount on current process PID
  * Merge "msm: adsprpc: null pointer check for sctx"
  * drivers: irqchip: qcom: Add MSM8937 pin data for MPM
  * ARM: qcom: Add board config support for qm215
  * soc: qcom: socinfo: Add supprot for QM215 QRD soc-id
  * soc: qcom: socinfo: Add support for msm8917 soc-id
  * soc: qcom: socinfo: add support for sdm429 and sdm439
  * soc: qcom: socinfo: Add support for msm8937 soc-id
  * arm/arm64: Kconfig: Add support for qm215
  * ARM: qcom: Add board config support for msm8917
  * arm64: Kconfig: Add ARCH_SDM439 and ARCH_SDM429 support
  * arm64: Kconfig: Add support for msm8937 on msm-4.9
  * msm: adsprpc: null pointer check for sctx
  * Merge "cnss2: Add delay for pci link training retries"
  * Merge "icnss: Add code to send qdss mode and sysfs interface for qdss"
  * f2fs: relocate f2fs_precache_extents()
  * f2fs: enforce the immutable flag on open files
  * f2fs: enhance to update i_mode and acl atomically in f2fs_setattr()
  * f2fs: fix to set inode->i_mode correctly for posix_acl_update_mode
  * f2fs: Replace expression with offsetof()
  * f2fs: handle unallocated section and zone on pinned/atgc
  * Merge "usb: gadget: composite: Add spinlock protection for usb string descriptor"
  * Merge "msm: kgsl: Protect the memdesc->gpuaddr in SVM use cases"
  * regulator: qpnp-lcdb: fix race between SC interrupt and lcdb enable
  * Merge "icnss: Add code to downdload qdss trace config"
  * msm: kgsl: Protect the memdesc->gpuaddr in SVM use cases
  * msm: kgsl: Stop using memdesc->usermem
  * net: sch_generic: fix the missing new qdisc assignment
  * usb: gadget: composite: Add spinlock protection for usb string descriptor
  * Merge "msm: kgsl: Correct the refcount on current process PID"
  * Merge "net: qualcomm: rmnet: memset before sending genl response"
  * msm: kgsl: add condition to check gpuhtw_llc support
  * icnss: Add code to send qdss mode and sysfs interface for qdss
  * icnss: Add code to downdload qdss trace config
  * Merge "leds: qpnp-flash-v2: Fix out-of-bound array access"
  * leds: qpnp-flash-v2: Fix out-of-bound array access
  * Merge "cnss2: Add code to fix a print error in update timestamp api"
  * mm: skip speculative path for non-anonymous COW faults
  * ARM: msm: Add board config support for 32 bit bengal apq iot
  * cnss2: Add code to fix a print error in update timestamp api
  * Merge "pci: msm: Add support for linkdown recovery on hot reset"
  * Merge "icnss2: change to avoid device crash if RF card is not present"
  * icnss2: change to avoid device crash if RF card is not present
  * Merge "mmc: sdhci-msm: read this reg IPCAT_MINOR_MASK to sdhc host"
  * mmc: sdhci-msm: read this reg IPCAT_MINOR_MASK to sdhc host
  * pci: msm: Add support for linkdown recovery on hot reset
  * Merge "spi: spi-geni-qcom: Calculate transfer timeout dynamically"
  * Merge "spi: spi-geni-qcom: Configure DFS index and clk before doing set rate"
  * Merge 718944c12951e67e831fc23140fbb37e9a64c273 on remote branch
  * Merge "serial: msm_geni_serial: Perform Abort sequence for cancel failure"
  * net: qrtr: print error case for qrtr receive
  * spi: spi-geni-qcom: Calculate transfer timeout dynamically
  * Merge "Revert "msm: kgsl: Add GPUCC register dumps to A6xx GPU snapshot""
  * spi: spi-geni-qcom: Configure DFS index and clk before doing set rate
  * cnss2: Add delay for pci link training retries
  * Merge "icnss2: Clear SMP2P data during SSR"
  * cnss2: Add code to handle idle-restart failure gracefully
  * icnss2: Clear SMP2P data during SSR
  * Revert "msm: kgsl: Add GPUCC register dumps to A6xx GPU snapshot"
  * net: qualcomm: rmnet: memset before sending genl response
  * Merge "diag: Add support for querying diagid for wpss subsystem"
  * Merge "spi: spi_qsd: Add ipc logging for spi driver"
  * diag: Add support for querying diagid for wpss subsystem
  * Merge "Merge android-4.19-stable.157 (8ee67bc) into msm-4.19"
  * Merge "wil6210: add ability to help debug tx latency"
  * Merge "scsi: ufshcd: Fix double release of hba"
  * wil6210: add ability to help debug tx latency
  * regulator: cpr: add snapshot of cpr-regulator driver
  * scsi: ufshcd: Fix double release of hba
  * Merge "walt: Add window rollover trace event"
  * Merge android-4.19-stable.157 (8ee67bc) into msm-4.19
  * walt: Add window rollover trace event
  * Merge "f2fs: should avoid inode eviction in synchronous path"
  * msm: kgsl: Correct the refcount on current process PID
  * Merge "ANDROID: vfs: d_canonical_path for stacked FS"
  * Merge "msm: kgsl: Poll a6x crashdumper register memory for status"
  * Merge "defconfig: kona: Enable USB UVC drivers"
  * Merge "USB: f_uac2: Update terminal types as mic and speaker"
  * Merge "usb: gadget: uvc: Update frame size as per frame type"
  * Merge "USB: gadget: Add support for superspeed plus for UAC2 & UVC"
  * USB: gadget: Add support for superspeed plus for UAC2 & UVC
  * Merge "msm: npu: Prevent unpaired power vote/unvote via sysfs node"
  * Merge "qseecom: use legacy command for slateapp"
  * Merge "usb: gadget: f_uvc: Fix video streaming quality issues"
  * Merge "cnss2: Fix KW issues for pointer check and variable init"
  * Merge "qdss_bridge: fix use-after-free is on cdev_put"
  * serial: msm_geni_serial: Perform Abort sequence for cancel failure
  * serial: msm_geni_serial: Correct start rx sequence
  * serial: msm_geni_serial: Pull RFR before stop_rx for uart
  * Revert "nl80211: fix non-split wiphy information"
  * qseecom: use legacy command for slateapp
  * Merge "usb: gadget: f_uvc: Fix crash on disconnect/connect during streaming"
  * Merge "msm: ipa: add check to see if pm client is not NULL"
  * USB: f_uac2: Update terminal types as mic and speaker
  * defconfig: kona: Enable USB UVC drivers
  * usb: gadget: f_uvc: Fix video streaming quality issues
  * usb: gadget: f_uvc: Fix crash on disconnect/connect during streaming
  * msm: npu: Prevent unpaired power vote/unvote via sysfs node
  * qdss_bridge: fix use-after-free is on cdev_put
  * Merge "trace: Fix race in trace_open and buffer resize call"
  * Merge "msm: ipa4: Add change to stop netdev"
  * HID: qvr: Remove device pointer on disconnect
  * defconfig: set CONFIG_HZ = 250 for Bengal
  * trace: Fix race in trace_open and buffer resize call
  * msm: ipa4: Add change to stop netdev
  * Merge "msm: ADSPRPC: Add check to avoid out of bound scenario"
  * Merge "USB: uvc_video: Check for return value before halt bulk endpoint"
  * Merge "USB: configfs: Clear deactivation flag in configfs_composite_unbind()"
  * USB: uvc_video: Check for return value before halt bulk endpoint
  * USB: configfs: Clear deactivation flag in configfs_composite_unbind()
  * Merge "msm: ipa3: Add change to increase timeout value"
  * Merge "msm: ADSPRPC: Reset debug buffer allocation check flag"
  * f2fs: compress: fix compression chksum
  * f2fs: fix shift-out-of-bounds in sanity_check_raw_super()
  * msm: ADSPRPC: Add check to avoid out of bound scenario
  * Merge "uapi: ipa: Support uc header proc ctx for DSCP insertion"
  * Merge "usb: dwc3-msm: Rectify 64 bit dma address programming for GSI"
  * Merge "trace: Fix race in trace_open and buffer resize call"
  * uapi: ipa: Support uc header proc ctx for DSCP insertion
  * usb: dwc3: core: Destroy ipc log context on remove
  * msm: ipa: add check to see if pm client is not NULL
  * cnss2: Fix KW issues for pointer check and variable init
  * ANDROID: vfs: d_canonical_path for stacked FS
  * msm: ipa3: Add change to increase timeout value
  * defconfig: arm64: msm: enable CONFIG_RB5_GPIOS_ENABLE
  * msm: driver: Add driver to enable RB5 GPIOs
  * f2fs: fix race of pending_pages in decompression
  * f2fs: fix to account inline xattr correctly during recovery
  * f2fs: inline: fix wrong inline inode stat
  * f2fs: inline: correct comment in f2fs_recover_inline_data
  * f2fs: don't check PAGE_SIZE again in sanity_check_raw_super()
  * f2fs: convert to F2FS_*_INO macro
  * Merge "defconfig: arm64: msm: enable CONFIG_RB5_FAN_CONTROLLER"
  * Merge "power: smb1355: Extend minoff-time and dead-time for SMB1355"
  * msm: ADSPRPC: Reset debug buffer allocation check flag
  * power: smb1355: Extend minoff-time and dead-time for SMB1355
  * Merge "msm: kgsl: Add apb_pclk to the clock list and increase max clock count"
  * Merge "f2fs: prepare a waiter before entering io_schedule"
  * defconfig: arm64: msm: enable CONFIG_RB5_FAN_CONTROLLER
  * msm: fan: Add driver to control fan on RB5
  * Merge "HID: qvr: Remove device pointer on disconnect"
  * HID: qvr: Remove device pointer on disconnect
  * Merge "power: smb1398-charger: Toggle UVLO-config on USB removal and shutdown"
  * usb: gadget: uvc: Update frame size as per frame type
  * usb: gadget: f_uvc: Fix incorrect frame indexing
  * usb: gadget: uvc: Add support for UVC 1.5
  * usb: gadget: Add support for UVC function
  * Merge "cnss2: Add api to get pci reg dump for hang data"
  * f2fs: prepare a waiter before entering io_schedule
  * f2fs: fix deadlock between quota writes and checkpoint
  * f2fs: introduce max_io_bytes, a sysfs entry, to limit bio size
  * f2fs: don't allow any writes on readonly mount
  * Initial commit
  * Merge "msm: ipa3: Add support PM api in ipav3"
  * cnss2: Add api to get pci reg dump for hang data
  * f2fs: avoid race condition for shrinker count
  * f2fs: add F2FS_IOC_DECOMPRESS_FILE and F2FS_IOC_COMPRESS_FILE
  * f2fs: add compress_mode mount option
  * f2fs: Remove unnecessary unlikely()
  * f2fs: init dirty_secmap incorrectly
  * f2fs: remove buffer_head which has 32bits limit
  * f2fs: fix wrong block count instead of bytes
  * f2fs: use new conversion functions between blks and bytes
  * f2fs: rename logical_to_blk and blk_to_logical
  * f2fs: fix kbytes written stat for multi-device case
  * f2fs: compress: support chksum
  * f2fs: fix to avoid REQ_TIME and CP_TIME collision
  * f2fs: change to use rwsem for cp_mutex
  * f2fs: Handle casefolding with Encryption
  * fscrypt: Have filesystems handle their d_ops
  * libfs: Add generic function for setting dentry_ops
  * f2fs: Remove the redundancy initialization
  * f2fs: remove writeback_inodes_sb in f2fs_remount
  * f2fs: fix double free of unicode map
  * f2fs: fix compat F2FS_IOC_{MOVE,GARBAGE_COLLECT}_RANGE
  * f2fs: avoid unneeded data copy in f2fs_ioc_move_range()
  * f2fs: add F2FS_IOC_SET_COMPRESS_OPTION ioctl
  * f2fs: add F2FS_IOC_GET_COMPRESS_OPTION ioctl
  * f2fs: move ioctl interface definitions to separated file
  * f2fs: fix to seek incorrect data offset in inline data file
  * f2fs: call f2fs_get_meta_page_retry for nat page
  * fscrypt: rename DCACHE_ENCRYPTED_NAME to DCACHE_NOKEY_NAME
  * fscrypt: don't call no-key names "ciphertext names"
  * fscrypt: export fscrypt_d_revalidate()
  * power: smb1398-charger: Toggle UVLO-config on USB removal and shutdown
  * Merge "fs: crypto: support IV_INO_LBLK_32 for legacy (V1) format"
  * msm: ipa3: Add support PM api in ipav3
  * Merge "msm: ipa3: Fix to unmap LOW LAT consumer pipe buffers"
  * Merge "msm: pcie: Dump PCIe registers for hang event"
  * fs: crypto: support IV_INO_LBLK_32 for legacy (V1) format
  * msm: ipa3: Fix to unmap LOW LAT consumer pipe buffers
  * msm: pcie: Dump PCIe registers for hang event
  * Merge 1d9cd56284e798ae33ec9040228279f062ab8107 on remote branch
  * trace: Fix race in trace_open and buffer resize call
  * Merge "msm: ipa4: Add change to stop netdev"
  * msm: ipa4: Add change to stop netdev
  * Merge "msm: fbdev: dp: send video notiification after audio teardown"
  * Merge "icnss2: Add code for SSR dump collection feature for moselle"
  * Merge "msm: ipa: Use kvfree instead of kfree"
  * msm: fbdev: dp: send video notiification after audio teardown
  * msm: kgsl: Poll a6x crashdumper register memory for status
  * usb: dwc3-msm: Rectify 64 bit dma address programming for GSI
  * Merge "msm: ADSPRPC: Enable ram dumps collection"
  * msm: ADSPRPC: Enable ram dumps collection
  * f2fs: should avoid inode eviction in synchronous path
  * Merge "msm: ipa3: Add check to validate rule_cnt"
  * Merge "i3c: i3c-master-qcom-geni: Add HW and FW version read support"
  * net: qrtr: Use cyclic idr allocator for port assignment
  * i3c: i3c-master-qcom-geni: Add HW and FW version read support
  * msm: ipa3: Add check to validate rule_cnt
  * Merge "i3c: i3c-master-qcom-geni: Manage driver suspend in late PM stage"
  * i3c: i3c-master-qcom-geni: Manage driver suspend in late PM stage
  * Merge "net: qrtr: Use cyclic idr allocator for port assignment"
  * net: qrtr: Use cyclic idr allocator for port assignment
  * Merge "mmc: core: change judgement for card busy detection"
  * Merge "msm: ADSPRPC: Handle third party applications"
  * Merge "HID: qvr: wake up after acknowledgment from viewer"
  * Merge "msm:ipa4: Fix race condition"
  * mmc: core: change judgement for card busy detection
  * msm: ADSPRPC: Handle third party applications
  * Merge "msm: kgsl: Check for gmu prealloc failure only in case of prealloc request"
  * msm:ipa4: Fix race condition
  * msm: kgsl: Check for gmu prealloc failure only in case of prealloc request
  * msm: ipa: Use kvfree instead of kfree
  * Merge "scsi : ufs-qcom: Add provision to set qcom specific supplies in LPM"
  * Merge "mhi: core: Increase RSC credit to 10"
  * Merge "cnss2: Check link status before setting wlaon_pwr_ctrl registers"
  * Merge "defconfig: arm64: msm: Enable USB RMNET & RNDIS using IPA over BAM2BAM"
  * Merge "wil6210: add max_mcs attribute"
  * Merge "serial: msm_geni_serial: Add enable/disable of dma irq bits"
  * Merge "drivers: phy: ufs: Remove the condition check to calibrate UFS Phy"
  * Merge "icnss: Update FW_INIT_DONE QMI indication with hang event params"
  * scsi : ufs-qcom: Add provision to set qcom specific supplies in LPM
  * fbdev: msm: update fb_release sequence
  * mhi: core: Increase RSC credit to 10
  * HID: qvr: wake up after acknowledgment from viewer
  * Merge "msm: ipa3: Fix to use proper clock timer"
  * Merge "usb: gadget: Add snapshot of USB RMNET Function driver"
  * Merge "msm: kgsl: Allow I/O coherency on imported buffers if we can"
  * icnss: Update FW_INIT_DONE QMI indication with hang event params
  * wil6210: add max_mcs attribute
  * Merge "usb: dwc3: msm: allow suspend in host mode"
  * cnss2: Check link status before setting wlaon_pwr_ctrl registers
  * defconfig: arm64: msm: Enable USB RMNET & RNDIS using IPA over BAM2BAM
  * msm: kgsl: Add apb_pclk to the clock list and increase max clock count
  * usb: gadget: Add snapshot of USB RMNET Function driver
  * drivers: soc: qcom: Add usb bam changes
  * usb: gadget: f_qc_rndis: Add RNDIS support using IPA over BAM2BAM
  * Merge "nl80211: fix NL80211_ATTR_HE_6GHZ_CAPABILITY usage"
  * Merge "defconfig: Sync with Android-4.19 configs"
  * Merge "clk: qcom: gcc: Update the halt flags for clocks on SDM660"
  * Merge "cnss2: Check for BT Enable GPIO in QCA6490 & QCA6390"
  * serial: msm_geni_serial: Add enable/disable of dma irq bits
  * serial: msm_geni_serial: Don't queue the rx dma buffer in stop rx
  * msm: kgsl: Allow I/O coherency on imported buffers if we can
  * Merge "cfg80211: reject HT/VHT capabilities on 6 GHz band"
  * Merge "mac80211: update HE operation fields to D3.0"
  * drivers: phy: ufs: Remove the condition check to calibrate UFS Phy
  * Merge "soc: qcom: minidump: Change the way to locate log_buf"
  * Merge "power: qpnp-qg: Expose CHARGE_COUNTER_SHADOW power supply property"
  * Merge "drivers: soc: qcom: Port bam dmux driver"
  * Merge "icnss2: Avoid race between soc wake release threads"
  * Reverting usb changes
  * defconfig: Sync with Android-4.19 configs
  * clk: qcom: gcc: Update the halt flags for clocks on SDM660
  * Merge "cfg80211: apply same mandatory rate flags for 5GHz and 6GHz"
  * Merge "cfg80211: ibss: use 11a mandatory rates for 6GHz band operation"
  * power: qpnp-qg: Expose CHARGE_COUNTER_SHADOW power supply property
  * Merge abde02226fa85217c38f924db5af4571fc060289 on remote branch
  * cnss2: Check for BT Enable GPIO in QCA6490 & QCA6390
  * icnss2: Avoid race between soc wake release threads
  * icnss2: Add code for SSR dump collection feature for moselle
  * nl80211: fix NL80211_ATTR_HE_6GHZ_CAPABILITY usage
  * cfg80211: Indicate support 6GHz band in kernel
  * cfg80211: treat 6 GHz channels as valid regardless of capability
  * cfg80211: require HE capabilities for 6 GHz band
  * cfg80211: reject HT/VHT capabilities on 6 GHz band
  * cfg80211: add and expose HE 6 GHz band capabilities
  * cfg80211: handle 6 GHz capability of new station
  * ieee80211: add HE ext EIDs and 6 GHz capability defines
  * ieee80211: add code to obtain and parse 6 GHz operation field
  * cfg80211: add a helper to identify 6 GHz PSCs
  * cfg80211: adapt to new channelization of the 6GHz band
  * cfg80211: fix 6 GHz frequencies to kHz
  * ieee80211: update HE IEs to D4.0 spec
  * mac80211: update HE IEs to D3.3
  * mac80211: update HE operation fields to D3.0
  * ieee80211: remove redundant leading zeroes
  * iwlwifi: split HE capabilities between AP and STA
  * netlink: add ethernet address policy types
  * netlink: add NLA_REJECT policy type
  * wireless: align to draft 11ax D3.0
  * mac80211: Add he_capa debugfs entry
  * cfg80211: express channels with a KHz component
  * cfg80211: Add support for 60GHz band channels 5 and 6
  * cfg80211: apply same mandatory rate flags for 5GHz and 6GHz
  * cfg80211: ibss: use 11a mandatory rates for 6GHz band operation
  * Merge "clk: qcom: clk-debug: Add support for debug clock API's"
  * Merge "clk: qcom: gpucc: Add support for new clock frequency for Lagoon"
  * Merge "usb: dwc3: gadget: Fix double add due to cleanup_cancelled_request"
  * Merge "icnss2: Add support for host triggered recovery"
  * clk: qcom: gpucc: Add support for new clock frequency for Lagoon
  * Merge "usb: dwc3: Ensure blocking_sync waits until host mode starts or stops"
  * soc: qcom: minidump: Change the way to locate log_buf
  * Merge "qcom-cpufreq: Removing lmh-dcvs parsing in ready callback"
  * qcom-cpufreq: Removing lmh-dcvs parsing in ready callback
  * usb: dwc3: msm: allow suspend in host mode
  * Merge "defconfig: sdm660: Enable CONFIG_HID_NINTENDO for sdm660"
  * Merge "Bluetooth: Implement a minimum off-time for AON discharge issue"
  * Merge "msm: ipa3: Add debug logs to check unregister netdev completion time"
  * Merge "net/ipv4: always honour route mtu during forwarding"
  * Merge "Correct the FM port numbers for Chk 3.x"
  * Merge "power: qpnp-smb5: Update legacy cable detection logic in bootup"
  * Merge "soc: qcom: Correct the module description for llcc-orchid"
  * Merge "cfg80211: use same IR permissive rules for 6GHz band"
  * Merge "i3c: i3c-master-qcom-geni: Handle timeout for DMA FSM reset"
  * Merge "scsi: ufs: Fix ufshcd_hold dead loop issue if error recovery is handing"
  * icnss2: Add support for host triggered recovery
  * Merge "msm: kgsl: Compare pid pointer instead of TGID for a new process"
  * fscrypt: Handle support for v1 encryption policy
  * Merge 4.19.157 into android-4.19-stable
  * Bluetooth: Implement a minimum off-time for AON discharge issue
  * defconfig: sdm660: Enable CONFIG_HID_NINTENDO for sdm660
  * defconfig: For support api_30 kernel changes
  * defconfig: Sync  with Android-4.19 configs
  * defconfig: Disable CRYPTO_MD4 config
  * defconfig: Enable VETH config
  * msm:adsprpc: Prevent use after free in fastrpc_set_process_info
  * Linux 4.19.157
  * powercap: restrict energy meter to root access
  * Revert "ANDROID: Kbuild, LLVMLinux: allow overriding clang target triple"
  * Correct the FM port numbers for Chk 3.x
  * Merge "Merge android-4.19-stable.152 (13abe23) into msm-4.19"
  * Merge 4.19.156 into android-4.19-stable
  * Linux 4.19.156
  * arm64: dts: marvell: espressobin: Add ethernet switch aliases
  * net: dsa: read mac address from DT for slave device
  * tools: perf: Fix build error in v4.19.y
  * perf/core: Fix a memory leak in perf_event_parse_addr_filter()
  * PM: runtime: Resume the device earlier in __device_release_driver()
  * Revert "ARC: entry: fix potential EFA clobber when TIF_SYSCALL_TRACE"
  * ARC: stack unwinding: avoid indefinite looping
  * usb: mtu3: fix panic in mtu3_gadget_stop()
  * USB: Add NO_LPM quirk for Kingston flash drive
  * USB: serial: option: add Telit FN980 composition 0x1055
  * USB: serial: option: add LE910Cx compositions 0x1203, 0x1230, 0x1231
  * USB: serial: option: add Quectel EC200T module support
  * USB: serial: cyberjack: fix write-URB completion race
  * serial: txx9: add missing platform_driver_unregister() on error in serial_txx9_init
  * serial: 8250_mtk: Fix uart_get_baud_rate warning
  * fork: fix copy_process(CLONE_PARENT) race with the exiting ->real_parent
  * vt: Disable KD_FONT_OP_COPY
  * ACPI: NFIT: Fix comparison to '-ENXIO'
  * drm/vc4: drv: Add error handding for bind
  * vsock: use ns_capable_noaudit() on socket create
  * scsi: core: Don't start concurrent async scan on same host
  * blk-cgroup: Pre-allocate tree node on blkg_conf_prep
  * blk-cgroup: Fix memleak on error path
  * of: Fix reserved-memory overlap detection
  * x86/kexec: Use up-to-dated screen_info copy to fill boot params
  * ARM: dts: sun4i-a10: fix cpu_alert temperature
  * futex: Handle transient "ownerless" rtmutex state correctly
  * tracing: Fix out of bounds write in get_trace_buf
  * ftrace: Handle tracing when switching between context
  * ftrace: Fix recursion check for NMI test
  * ring-buffer: Fix recursion protection transitions between interrupt context
  * gfs2: Wake up when sd_glock_disposal becomes zero
  * mm: always have io_remap_pfn_range() set pgprot_decrypted()
  * kthread_worker: prevent queuing delayed work from timer_fn when it is being canceled
  * lib/crc32test: remove extra local_irq_disable/enable
  * mm: mempolicy: fix potential pte_unmap_unlock pte error
  * ALSA: usb-audio: Add implicit feedback quirk for MODX
  * ALSA: usb-audio: Add implicit feedback quirk for Qu-16
  * ALSA: usb-audio: add usb vendor id as DSD-capable for Khadas devices
  * ALSA: usb-audio: Add implicit feedback quirk for Zoom UAC-2
  * Fonts: Replace discarded const qualifier
  * btrfs: tree-checker: fix the error message for transid error
  * btrfs: tree-checker: Verify inode item
  * btrfs: tree-checker: Enhance chunk checker to validate chunk profile
  * btrfs: tree-checker: Fix wrong check on max devid
  * btrfs: tree-checker: Verify dev item
  * btrfs: tree-checker: Check chunk item at tree block read time
  * btrfs: tree-checker: Make btrfs_check_chunk_valid() return EUCLEAN instead of EIO
  * btrfs: tree-checker: Make chunk item checker messages more readable
  * btrfs: Move btrfs_check_chunk_valid() to tree-check.[ch] and export it
  * btrfs: Don't submit any btree write bio if the fs has errors
  * Btrfs: fix unwritten extent buffers and hangs on future writeback attempts
  * btrfs: extent_io: add proper error handling to lock_extent_buffer_for_io()
  * btrfs: extent_io: Handle errors better in btree_write_cache_pages()
  * btrfs: extent_io: Handle errors better in extent_write_full_page()
  * btrfs: flush write bio if we loop in extent_write_cache_pages
  * Revert "btrfs: flush write bio if we loop in extent_write_cache_pages"
  * btrfs: extent_io: Move the BUG_ON() in flush_write_bio() one level up
  * btrfs: extent_io: Kill the forward declaration of flush_write_bio
  * blktrace: fix debugfs use after free
  * sfp: Fix error handing in sfp_probe()
  * sctp: Fix COMM_LOST/CANT_STR_ASSOC err reporting on big-endian platforms
  * net: usb: qmi_wwan: add Telit LE910Cx 0x1230 composition
  * gianfar: Account for Tx PTP timestamp in the skb headroom
  * gianfar: Replace skb_realloc_headroom with skb_cow_head for PTP
  * chelsio/chtls: fix always leaking ctrl_skb
  * chelsio/chtls: fix memory leaks caused by a race
  * cadence: force nonlinear buffers to be cloned
  * ptrace: fix task_join_group_stop() for the case when current is traced
  * tipc: fix use-after-free in tipc_bcast_get_mode
  * drm/i915: Break up error capture compression loops with cond_resched()
  * net/ipv4: always honour route mtu during forwarding
  * Merge "Revert "clk: Evict unregistered clks from parent caches""
  * usb: dwc3: Ensure blocking_sync waits until host mode starts or stops
  * usb: dwc3: gadget: Fix double add due to cleanup_cancelled_request
  * ANDROID: fuse: Add support for d_canonical_path
  * ANDROID: vfs: add d_canonical_path for stacked filesystem support
  * msm: kgsl: Compare pid pointer instead of TGID for a new process
  * clk: qcom: clk-debug: Add support for debug clock API's
  * cfg80211: use same IR permissive rules for 6GHz band
  * cfg80211: add 6GHz in code handling array with NUM_NL80211_BANDS entries
  * cfg80211: extend ieee80211_operating_class_to_band() for 6GHz
  * cfg80211: util: add 6GHz channel to freq conversion and vice versa
  * cfg80211: add 6GHz UNII band definitions
  * nl80211: add 6GHz band definition to enum nl80211_band
  * ANDROID: Temporarily disable XFRM_USER_COMPAT filtering
  * Merge 4.19.155 into android-4.19-stable
  * msm: ipa3: Add debug logs to check unregister netdev completion time
  * Linux 4.19.155
  * staging: octeon: Drop on uncorrectable alignment or FCS error
  * staging: octeon: repair "fixed-link" support
  * staging: comedi: cb_pcidas: Allow 2-channel commands for AO subdevice
  * KVM: arm64: Fix AArch32 handling of DBGD{CCINT,SCRext} and DBGVCR
  * device property: Don't clear secondary pointer for shared primary firmware node
  * device property: Keep secondary firmware node secondary by type
  * ARM: s3c24xx: fix missing system reset
  * ARM: samsung: fix PM debug build with DEBUG_LL but !MMU
  * arm: dts: mt7623: add missing pause for switchport
  * hil/parisc: Disable HIL driver when it gets stuck
  * cachefiles: Handle readpage error correctly
  * arm64: berlin: Select DW_APB_TIMER_OF
  * tty: make FONTX ioctl use the tty pointer they were actually passed
  * rtc: rx8010: don't modify the global rtc ops
  * drm/ttm: fix eviction valuable range check.
  * ext4: fix invalid inode checksum
  * ext4: fix error handling code in add_new_gdb
  * ext4: fix leaking sysfs kobject after failed mount
  * vringh: fix __vringh_iov() when riov and wiov are different
  * ring-buffer: Return 0 on success from ring_buffer_resize()
  * 9P: Cast to loff_t before multiplying
  * libceph: clear con->out_msg on Policy::stateful_server faults
  * ceph: promote to unsigned long long before shifting
  * drm/amd/display: Don't invoke kgdb_breakpoint() unconditionally
  * drm/amdgpu: don't map BO in reserved region
  * i2c: imx: Fix external abort on interrupt in exit paths
  * ia64: fix build error with !COREDUMP
  * ubi: check kthread_should_stop() after the setting of task state
  * perf python scripting: Fix printable strings in python3 scripts
  * ubifs: dent: Fix some potential memory leaks while iterating entries
  * NFSD: Add missing NFSv2 .pc_func methods
  * NFSv4.2: support EXCHGID4_FLAG_SUPP_FENCE_OPS 4.2 EXCHANGE_ID flag
  * powerpc: Fix undetected data corruption with P9N DD2.1 VSX CI load emulation
  * powerpc/powernv/elog: Fix race while processing OPAL error log event.
  * powerpc: Warn about use of smt_snooze_delay
  * powerpc/rtas: Restrict RTAS requests from userspace
  * s390/stp: add locking to sysfs functions
  * powerpc/drmem: Make lmb_size 64 bit
  * iio:gyro:itg3200: Fix timestamp alignment and prevent data leak.
  * iio:adc:ti-adc12138 Fix alignment issue with timestamp
  * iio:adc:ti-adc0832 Fix alignment issue with timestamp
  * iio:light:si1145: Fix timestamp alignment and prevent data leak.
  * dmaengine: dma-jz4780: Fix race in jz4780_dma_tx_status
  * udf: Fix memory leak when mounting
  * HID: wacom: Avoid entering wacom_wac_pen_report for pad / battery
  * vt: keyboard, extend func_buf_lock to readers
  * vt: keyboard, simplify vt_kdgkbsent
  * drm/i915: Force VT'd workarounds when running as a guest OS
  * usb: host: fsl-mph-dr-of: check return of dma_set_mask()
  * usb: typec: tcpm: reset hard_reset_count for any disconnect
  * usb: cdc-acm: fix cooldown mechanism
  * usb: dwc3: core: don't trigger runtime pm when remove driver
  * usb: dwc3: core: add phy cleanup for probe error handling
  * usb: dwc3: gadget: Check MPS of the request length
  * usb: dwc3: ep0: Fix ZLP for OUT ep0 requests
  * usb: xhci: Workaround for S3 issue on AMD SNPS 3.0 xHC
  * btrfs: fix use-after-free on readahead extent after failure to create it
  * btrfs: cleanup cow block on error
  * btrfs: use kvzalloc() to allocate clone_roots in btrfs_ioctl_send()
  * btrfs: send, recompute reference path after orphanization of a directory
  * btrfs: reschedule if necessary when logging directory items
  * btrfs: improve device scanning messages
  * btrfs: qgroup: fix wrong qgroup metadata reserve for delayed inode
  * scsi: qla2xxx: Fix crash on session cleanup with unload
  * scsi: mptfusion: Fix null pointer dereferences in mptscsih_remove()
  * w1: mxc_w1: Fix timeout resolution problem leading to bus error
  * acpi-cpufreq: Honor _PSD table setting on new AMD CPUs
  * ACPI: debug: don't allow debugging when ACPI is disabled
  * ACPI: video: use ACPI backlight for HP 635 Notebook
  * ACPI / extlog: Check for RDMSR failure
  * ACPI: button: fix handling lid state changes when input device closed
  * NFS: fix nfs_path in case of a rename retry
  * fs: Don't invalidate page buffers in block_write_full_page()
  * media: uvcvideo: Fix uvc_ctrl_fixup_xu_info() not having any effect
  * leds: bcm6328, bcm6358: use devres LED registering function
  * perf/x86/amd/ibs: Fix raw sample data accumulation
  * perf/x86/amd/ibs: Don't include randomized bits in get_ibs_op_count()
  * mmc: sdhci-acpi: AMDI0040: Set SDHCI_QUIRK2_PRESET_VALUE_BROKEN
  * md/raid5: fix oops during stripe resizing
  * nvme-rdma: fix crash when connect rejected
  * sgl_alloc_order: fix memory leak
  * nbd: make the config put is called before the notifying the waiter
  * ARM: dts: s5pv210: remove dedicated 'audio-subsystem' node
  * ARM: dts: s5pv210: move PMU node out of clock controller
  * ARM: dts: s5pv210: remove DMA controller bus node name to fix dtschema warnings
  * memory: emif: Remove bogus debugfs error handling
  * ARM: dts: omap4: Fix sgx clock rate for 4430
  * arm64: dts: renesas: ulcb: add full-pwr-cycle-in-suspend into eMMC nodes
  * cifs: handle -EINTR in cifs_setattr
  * gfs2: add validation checks for size of superblock
  * ext4: Detect already used quota file early
  * drivers: watchdog: rdc321x_wdt: Fix race condition bugs
  * net: 9p: initialize sun_server.sun_path to have addr's value only when addr is valid
  * clk: ti: clockdomain: fix static checker warning
  * rpmsg: glink: Use complete_all for open states
  * bnxt_en: Log unknown link speed appropriately.
  * md/bitmap: md_bitmap_get_counter returns wrong blocks
  * btrfs: fix replace of seed device
  * drm/amd/display: HDMI remote sink need mode validation for Linux
  * power: supply: test_power: add missing newlines when printing parameters by sysfs
  * bus/fsl_mc: Do not rely on caller to provide non NULL mc_io
  * drivers/net/wan/hdlc_fr: Correctly handle special skb->protocol values
  * ACPI: Add out of bounds and numa_off protections to pxm_to_node()
  * xfs: don't free rt blocks when we're doing a REMAP bunmapi call
  * arm64/mm: return cpu_all_mask when node is NUMA_NO_NODE
  * usb: xhci: omit duplicate actions when suspending a runtime suspended host.
  * uio: free uio id after uio file node is freed
  * USB: adutux: fix debugging
  * cpufreq: sti-cpufreq: add stih418 support
  * riscv: Define AT_VECTOR_SIZE_ARCH for ARCH_DLINFO
  * media: uvcvideo: Fix dereference of out-of-bound list iterator
  * kgdb: Make "kgdbcon" work properly with "kgdb_earlycon"
  * ia64: kprobes: Use generic kretprobe trampoline handler
  * printk: reduce LOG_BUF_SHIFT range for H8300
  * arm64: topology: Stop using MPIDR for topology information
  * drm/bridge/synopsys: dsi: add support for non-continuous HS clock
  * mmc: via-sdmmc: Fix data race bug
  * media: imx274: fix frame interval handling
  * media: tw5864: check status of tw5864_frameinterval_get
  * usb: typec: tcpm: During PR_SWAP, source caps should be sent only after tSwapSourceStart
  * media: platform: Improve queue set up flow for bug fixing
  * media: videodev2.h: RGB BT2020 and HSV are always full range
  * drm/brige/megachips: Add checking if ge_b850v3_lvds_init() is working correctly
  * ath10k: fix VHT NSS calculation when STBC is enabled
  * ath10k: start recovery process when payload length exceeds max htc length for sdio
  * video: fbdev: pvr2fb: initialize variables
  * xfs: fix realtime bitmap/summary file truncation when growing rt volume
  * power: supply: bq27xxx: report "not charging" on all types
  * ARM: 8997/2: hw_breakpoint: Handle inexact watchpoint addresses
  * um: change sigio_spinlock to a mutex
  * f2fs: fix to check segment boundary during SIT page readahead
  * f2fs: fix uninit-value in f2fs_lookup
  * f2fs: add trace exit in exception path
  * sparc64: remove mm_cpumask clearing to fix kthread_use_mm race
  * powerpc: select ARCH_WANT_IRQS_OFF_ACTIVATE_MM
  * mm: fix exec activate_mm vs TLB shootdown and lazy tlb switching race
  * powerpc/powernv/smp: Fix spurious DBG() warning
  * futex: Fix incorrect should_fail_futex() handling
  * ata: sata_nv: Fix retrieving of active qcs
  * RDMA/qedr: Fix memory leak in iWARP CM
  * mlxsw: core: Fix use-after-free in mlxsw_emad_trans_finish()
  * x86/unwind/orc: Fix inactive tasks with stack pointer in %sp on GCC 10 compiled kernels
  * xen/events: block rogue events for some time
  * xen/events: defer eoi in case of excessive number of events
  * xen/events: use a common cpu hotplug hook for event channels
  * xen/events: switch user event channels to lateeoi model
  * xen/pciback: use lateeoi irq binding
  * xen/pvcallsback: use lateeoi irq binding
  * xen/scsiback: use lateeoi irq binding
  * xen/netback: use lateeoi irq binding
  * xen/blkback: use lateeoi irq binding
  * xen/events: add a new "late EOI" evtchn framework
  * xen/events: fix race in evtchn_fifo_unmask()
  * xen/events: add a proper barrier to 2-level uevent unmasking
  * xen/events: avoid removing an event channel while handling it
  * xen/events: don't use chip_data for legacy IRQs
  * Revert "block: ratelimit handle_bad_sector() message"
  * fscrypt: fix race where ->lookup() marks plaintext dentry as ciphertext
  * fscrypt: only set dentry_operations on ciphertext dentries
  * fs, fscrypt: clear DCACHE_ENCRYPTED_NAME when unaliasing directory
  * fscrypt: fix race allowing rename() and link() of ciphertext dentries
  * fscrypt: clean up and improve dentry revalidation
  * fscrypt: return -EXDEV for incompatible rename or link into encrypted dir
  * ata: sata_rcar: Fix DMA boundary mask
  * serial: pl011: Fix lockdep splat when handling magic-sysrq interrupt
  * mtd: lpddr: Fix bad logic in print_drs_error
  * RDMA/addr: Fix race with netevent_callback()/rdma_addr_cancel()
  * cxl: Rework error message for incompatible slots
  * p54: avoid accessing the data mapped to streaming DMA
  * evm: Check size of security.evm before using it
  * bpf: Fix comment for helper bpf_current_task_under_cgroup()
  * fuse: fix page dereference after free
  * x86/xen: disable Firmware First mode for correctable memory errors
  * arch/x86/amd/ibs: Fix re-arming IBS Fetch
  * cxgb4: set up filter action after rewrites
  * r8169: fix issue with forced threading in combination with shared interrupts
  * tipc: fix memory leak caused by tipc_buf_append()
  * tcp: Prevent low rmem stalls with SO_RCVLOWAT.
  * ravb: Fix bit fields checking in ravb_hwtstamp_get()
  * netem: fix zero division in tabledist
  * mlxsw: core: Fix memory leak on module removal
  * gtp: fix an use-before-init in gtp_newlink()
  * chelsio/chtls: fix tls record info to user
  * chelsio/chtls: fix memory leaks in CPL handlers
  * chelsio/chtls: fix deadlock issue
  * efivarfs: Replace invalid slashes with exclamation marks in dentries.
  * x86/PCI: Fix intel_mid_pci.c build error when ACPI is not enabled
  * arm64: link with -z norelro regardless of CONFIG_RELOCATABLE
  * arm64: Run ARCH_WORKAROUND_1 enabling code on all CPUs
  * scripts/setlocalversion: make git describe output more reliable
  * objtool: Support Clang non-section symbols in ORC generation
  * Merge 6d79a96abbe7501e78771abebd639d858d1e2d71 on remote branch
  * Revert "clk: Evict unregistered clks from parent caches"
  * ANDROID: GKI: Enable DEBUG_INFO_DWARF4
  * timekeeping/vsyscall: Prevent math overflow in BOOTTIME update
  * UPSTREAM: mm/sl[uo]b: export __kmalloc_track(_node)_caller
  * Merge "defconfig: Enable CONFIG_LEGACY_ENERGY_MODEL_DT for sdm660"
  * Merge "fscrypt: Handle support for v1 encryption policy"
  * Merge "i3c: i3c-master-qcom-geni: Log error if DMA mode fails"
  * Merge "msm: kgsl: Compare pid pointer instead of TGID for a new process"
  * Merge "qbt_handler: Memset userspace struct to zero"
  * BACKPORT: xfrm/compat: Translate 32-bit user_policy from sockptr
  * BACKPORT: xfrm/compat: Add 32=>64-bit messages translator
  * UPSTREAM: xfrm/compat: Attach xfrm dumps to 64=>32 bit translator
  * UPSTREAM: xfrm/compat: Add 64=>32-bit messages translator
  * BACKPORT: xfrm: Provide API to register translator module
  * ANDROID: Publish uncompressed Image on aarch64
  * FROMLIST: crypto: arm64/poly1305-neon - reorder PAC authentication with SP update
  * UPSTREAM: crypto: arm64/chacha - fix chacha_4block_xor_neon() for big endian
  * UPSTREAM: crypto: arm64/chacha - fix hchacha_block_neon() for big endian
  * msm: ipa: fix the use-after-free on qmi framework in ssr scenario
  * Merge 4.19.154 into android-4.19-stable
  * Linux 4.19.154
  * usb: gadget: f_ncm: allow using NCM in SuperSpeed Plus gadgets.
  * eeprom: at25: set minimum read/write access stride to 1
  * USB: cdc-wdm: Make wdm_flush() interruptible and add wdm_fsync().
  * usb: cdc-acm: add quirk to blacklist ETAS ES58X devices
  * tty: serial: fsl_lpuart: fix lpuart32_poll_get_char
  * net: korina: cast KSEG0 address to pointer in kfree
  * ath10k: check idx validity in __ath10k_htt_rx_ring_fill_n()
  * scsi: ufs: ufs-qcom: Fix race conditions caused by ufs_qcom_testbus_config()
  * usb: core: Solve race condition in anchor cleanup functions
  * brcm80211: fix possible memleak in brcmf_proto_msgbuf_attach
  * mwifiex: don't call del_timer_sync() on uninitialized timer
  * reiserfs: Fix memory leak in reiserfs_parse_options()
  * ipvs: Fix uninit-value in do_ip_vs_set_ctl()
  * tty: ipwireless: fix error handling
  * scsi: qedi: Fix list_del corruption while removing active I/O
  * scsi: qedi: Protect active command list to avoid list corruption
  * Fix use after free in get_capset_info callback.
  * rtl8xxxu: prevent potential memory leak
  * brcmsmac: fix memory leak in wlc_phy_attach_lcnphy
  * scsi: ibmvfc: Fix error return in ibmvfc_probe()
  * Bluetooth: Only mark socket zapped after unlocking
  * usb: ohci: Default to per-port over-current protection
  * xfs: make sure the rt allocator doesn't run off the end
  * reiserfs: only call unlock_new_inode() if I_NEW
  * misc: rtsx: Fix memory leak in rtsx_pci_probe
  * ath9k: hif_usb: fix race condition between usb_get_urb() and usb_kill_anchored_urbs()
  * can: flexcan: flexcan_chip_stop(): add error handling and propagate error value
  * usb: dwc3: simple: add support for Hikey 970
  * USB: cdc-acm: handle broken union descriptors
  * udf: Avoid accessing uninitialized data on failed inode read
  * udf: Limit sparing table size
  * usb: gadget: function: printer: fix use-after-free in __lock_acquire
  * misc: vop: add round_up(x,4) for vring_size to avoid kernel panic
  * mic: vop: copy data to kernel space then write to io memory
  * scsi: target: core: Add CONTROL field for trace events
  * scsi: mvumi: Fix error return in mvumi_io_attach()
  * PM: hibernate: remove the bogus call to get_gendisk() in software_resume()
  * mac80211: handle lack of sband->bitrates in rates
  * ip_gre: set dev->hard_header_len and dev->needed_headroom properly
  * ntfs: add check for mft record size in superblock
  * media: venus: core: Fix runtime PM imbalance in venus_probe
  * fs: dlm: fix configfs memory leak
  * media: saa7134: avoid a shift overflow
  * mmc: sdio: Check for CISTPL_VERS_1 buffer size
  * media: uvcvideo: Ensure all probed info is returned to v4l2
  * media: media/pci: prevent memory leak in bttv_probe
  * media: bdisp: Fix runtime PM imbalance on error
  * media: platform: sti: hva: Fix runtime PM imbalance on error
  * media: platform: s3c-camif: Fix runtime PM imbalance on error
  * media: vsp1: Fix runtime PM imbalance on error
  * media: exynos4-is: Fix a reference count leak
  * media: exynos4-is: Fix a reference count leak due to pm_runtime_get_sync
  * media: exynos4-is: Fix several reference count leaks due to pm_runtime_get_sync
  * media: sti: Fix reference count leaks
  * media: st-delta: Fix reference count leak in delta_run_work
  * media: ati_remote: sanity check for both endpoints
  * media: firewire: fix memory leak
  * crypto: ccp - fix error handling
  * block: ratelimit handle_bad_sector() message
  * i2c: core: Restore acpi_walk_dep_device_list() getting called after registering the ACPI i2c devs
  * perf: correct SNOOPX field offset
  * sched/features: Fix !CONFIG_JUMP_LABEL case
  * NTB: hw: amd: fix an issue about leak system resources
  * nvmet: fix uninitialized work for zero kato
  * powerpc/powernv/dump: Fix race while processing OPAL dump
  * arm64: dts: zynqmp: Remove additional compatible string for i2c IPs
  * ARM: dts: owl-s500: Fix incorrect PPI interrupt specifiers
  * arm64: dts: qcom: msm8916: Fix MDP/DSI interrupts
  * arm64: dts: qcom: pm8916: Remove invalid reg size from wcd_codec
  * memory: fsl-corenet-cf: Fix handling of platform_get_irq() error
  * memory: omap-gpmc: Fix build error without CONFIG_OF
  * memory: omap-gpmc: Fix a couple off by ones
  * ARM: dts: sun8i: r40: bananapi-m2-ultra: Fix dcdc1 regulator
  * ARM: dts: imx6sl: fix rng node
  * netfilter: nf_fwd_netdev: clear timestamp in forwarding path
  * netfilter: conntrack: connection timeout after re-register
  * KVM: x86: emulating RDPID failure shall return #UD rather than #GP
  * Input: sun4i-ps2 - fix handling of platform_get_irq() error
  * Input: twl4030_keypad - fix handling of platform_get_irq() error
  * Input: omap4-keypad - fix handling of platform_get_irq() error
  * Input: ep93xx_keypad - fix handling of platform_get_irq() error
  * Input: stmfts - fix a & vs && typo
  * Input: imx6ul_tsc - clean up some errors in imx6ul_tsc_resume()
  * SUNRPC: fix copying of multiple pages in gss_read_proxy_verf()
  * vfio iommu type1: Fix memory leak in vfio_iommu_type1_pin_pages
  * vfio/pci: Clear token on bypass registration failure
  * ext4: limit entries returned when counting fsmap records
  * svcrdma: fix bounce buffers for unaligned offsets and multiple pages
  * watchdog: sp5100: Fix definition of EFCH_PM_DECODEEN3
  * watchdog: Use put_device on error
  * watchdog: Fix memleak in watchdog_cdev_register
  * clk: bcm2835: add missing release if devm_clk_hw_register fails
  * clk: at91: clk-main: update key before writing AT91_CKGR_MOR
  * clk: rockchip: Initialize hw to error to avoid undefined behavior
  * pwm: img: Fix null pointer access in probe
  * rpmsg: smd: Fix a kobj leak in in qcom_smd_parse_edge()
  * PCI: iproc: Set affinity mask on MSI interrupts
  * i2c: rcar: Auto select RESET_CONTROLLER
  * mailbox: avoid timer start from callback
  * rapidio: fix the missed put_device() for rio_mport_add_riodev
  * rapidio: fix error handling path
  * ramfs: fix nommu mmap with gaps in the page cache
  * lib/crc32.c: fix trivial typo in preprocessor condition
  * f2fs: wait for sysfs kobject removal before freeing f2fs_sb_info
  * IB/rdmavt: Fix sizeof mismatch
  * cpufreq: powernv: Fix frame-size-overflow in powernv_cpufreq_reboot_notifier
  * powerpc/perf/hv-gpci: Fix starting index value
  * powerpc/perf: Exclude pmc5/6 from the irrelevant PMU group constraints
  * overflow: Include header file with SIZE_MAX declaration
  * kdb: Fix pager search for multi-line strings
  * RDMA/hns: Fix missing sq_sig_type when querying QP
  * RDMA/hns: Set the unsupported wr opcode
  * perf intel-pt: Fix "context_switch event has no tid" error
  * RDMA/cma: Consolidate the destruction of a cma_multicast in one place
  * RDMA/cma: Remove dead code for kernel rdmacm multicast
  * powerpc/64s/radix: Fix mm_cpumask trimming race vs kthread_use_mm
  * powerpc/tau: Disable TAU between measurements
  * powerpc/tau: Check processor type before enabling TAU interrupt
  * fscrypt: Handle support for v1 encryption policy
  * ANDROID: GKI: update the ABI xml
  * spi: spi_qsd: Add ipc logging for spi driver
  * Merge 4.19.153 into android-4.19-stable
  * Linux 4.19.153
  * powerpc/tau: Remove duplicated set_thresholds() call
  * powerpc/tau: Convert from timer to workqueue
  * powerpc/tau: Use appropriate temperature sample interval
  * RDMA/qedr: Fix inline size returned for iWARP
  * RDMA/qedr: Fix use of uninitialized field
  * xfs: fix high key handling in the rt allocator's query_range function
  * xfs: limit entries returned when counting fsmap records
  * arc: plat-hsdk: fix kconfig dependency warning when !RESET_CONTROLLER
  * ARM: 9007/1: l2c: fix prefetch bits init in L2X0_AUX_CTRL using DT values
  * mtd: mtdoops: Don't write panic data twice
  * powerpc/pseries: explicitly reschedule during drmem_lmb list traversal
  * mtd: lpddr: fix excessive stack usage with clang
  * RDMA/ucma: Add missing locking around rdma_leave_multicast()
  * RDMA/ucma: Fix locking for ctx->events_reported
  * powerpc/icp-hv: Fix missing of_node_put() in success path
  * powerpc/pseries: Fix missing of_node_put() in rng_init()
  * IB/mlx4: Adjust delayed work when a dup is observed
  * IB/mlx4: Fix starvation in paravirt mux/demux
  * mm, oom_adj: don't loop through tasks in __set_oom_adj when not necessary
  * mm/memcg: fix device private memcg accounting
  * netfilter: nf_log: missing vlan offload tag and proto
  * net: korina: fix kfree of rx/tx descriptor array
  * ipvs: clear skb->tstamp in forwarding path
  * mwifiex: fix double free
  * platform/x86: mlx-platform: Remove PSU EEPROM configuration
  * scsi: be2iscsi: Fix a theoretical leak in beiscsi_create_eqs()
  * scsi: target: tcmu: Fix warning: 'page' may be used uninitialized
  * usb: dwc2: Fix INTR OUT transfers in DDMA mode.
  * nl80211: fix non-split wiphy information
  * usb: gadget: u_ether: enable qmult on SuperSpeed Plus as well
  * usb: gadget: f_ncm: fix ncm_bitrate for SuperSpeed and above.
  * iwlwifi: mvm: split a print to avoid a WARNING in ROC
  * mfd: sm501: Fix leaks in probe()
  * net: enic: Cure the enic api locking trainwreck
  * qtnfmac: fix resource leaks on unsupported iftype error return path
  * HID: hid-input: fix stylus battery reporting
  * slimbus: qcom-ngd-ctrl: disable ngd in qmi server down callback
  * slimbus: core: do not enter to clock pause mode in core
  * slimbus: core: check get_addr before removing laddr ida
  * quota: clear padding in v2r1_mem2diskdqb()
  * usb: dwc2: Fix parameter type in function pointer prototype
  * ALSA: seq: oss: Avoid mutex lock for a long-time ioctl
  * misc: mic: scif: Fix error handling path
  * ath6kl: wmi: prevent a shift wrapping bug in ath6kl_wmi_delete_pstream_cmd()
  * net: dsa: rtl8366rb: Support all 4096 VLANs
  * net: dsa: rtl8366: Skip PVID setting if not requested
  * net: dsa: rtl8366: Refactor VLAN/PVID init
  * net: dsa: rtl8366: Check validity of passed VLANs
  * cpufreq: armada-37xx: Add missing MODULE_DEVICE_TABLE
  * net: stmmac: use netif_tx_start|stop_all_queues() function
  * net/mlx5: Don't call timecounter cyc2time directly from 1PPS flow
  * pinctrl: mcp23s08: Fix mcp23x17 precious range
  * pinctrl: mcp23s08: Fix mcp23x17_regmap initialiser
  * HID: roccat: add bounds checking in kone_sysfs_write_settings()
  * video: fbdev: radeon: Fix memleak in radeonfb_pci_register
  * video: fbdev: sis: fix null ptr dereference
  * video: fbdev: vga16fb: fix setting of pixclock because a pass-by-value error
  * drivers/virt/fsl_hypervisor: Fix error handling path
  * pwm: lpss: Add range limit check for the base_unit register value
  * pwm: lpss: Fix off by one error in base_unit math in pwm_lpss_prepare()
  * pty: do tty_flip_buffer_push without port->lock in pty_write
  * tty: hvcs: Don't NULL tty->driver_data until hvcs_cleanup()
  * tty: serial: earlycon dependency
  * VMCI: check return value of get_user_pages_fast() for errors
  * backlight: sky81452-backlight: Fix refcount imbalance on error
  * scsi: csiostor: Fix wrong return value in csio_hw_prep_fw()
  * scsi: qla2xxx: Fix wrong return value in qla_nvme_register_hba()
  * scsi: qla4xxx: Fix an error handling path in 'qla4xxx_get_host_stats()'
  * drm/gma500: fix error check
  * staging: rtl8192u: Do not use GFP_KERNEL in atomic context
  * mwifiex: Do not use GFP_KERNEL in atomic context
  * brcmfmac: check ndev pointer
  * ASoC: qcom: lpass-cpu: fix concurrency issue
  * ASoC: qcom: lpass-platform: fix memory leak
  * wcn36xx: Fix reported 802.11n rx_highest rate wcn3660/wcn3680
  * ath10k: Fix the size used in a 'dma_free_coherent()' call in an error handling path
  * ath9k: Fix potential out of bounds in ath9k_htc_txcompletion_cb()
  * ath6kl: prevent potential array overflow in ath6kl_add_new_sta()
  * Bluetooth: hci_uart: Cancel init work before unregistering
  * ath10k: provide survey info as accumulated data
  * spi: spi-s3c64xx: Check return values
  * spi: spi-s3c64xx: swap s3c64xx_spi_set_cs() and s3c64xx_enable_datapath()
  * pinctrl: bcm: fix kconfig dependency warning when !GPIOLIB
  * regulator: resolve supply after creating regulator
  * media: ti-vpe: Fix a missing check and reference count leak
  * media: stm32-dcmi: Fix a reference count leak
  * media: s5p-mfc: Fix a reference count leak
  * media: camss: Fix a reference count leak.
  * media: platform: fcp: Fix a reference count leak.
  * media: rockchip/rga: Fix a reference count leak.
  * media: rcar-vin: Fix a reference count leak.
  * media: tc358743: cleanup tc358743_cec_isr
  * media: tc358743: initialize variable
  * media: mx2_emmaprp: Fix memleak in emmaprp_probe
  * cypto: mediatek - fix leaks in mtk_desc_ring_alloc
  * hwmon: (pmbus/max34440) Fix status register reads for MAX344{51,60,61}
  * crypto: omap-sham - fix digcnt register handling with export/import
  * media: omap3isp: Fix memleak in isp_probe
  * media: uvcvideo: Silence shift-out-of-bounds warning
  * media: uvcvideo: Set media controller entity functions
  * media: m5mols: Check function pointer in m5mols_sensor_power
  * media: Revert "media: exynos4-is: Add missed check for pinctrl_lookup_state()"
  * media: tuner-simple: fix regression in simple_set_radio_freq
  * crypto: picoxcell - Fix potential race condition bug
  * crypto: ixp4xx - Fix the size used in a 'dma_free_coherent()' call
  * crypto: mediatek - Fix wrong return value in mtk_desc_ring_alloc()
  * crypto: algif_skcipher - EBUSY on aio should be an error
  * x86/events/amd/iommu: Fix sizeof mismatch
  * x86/nmi: Fix nmi_handle() duration miscalculation
  * drivers/perf: xgene_pmu: Fix uninitialized resource struct
  * x86/fpu: Allow multiple bits in clearcpuid= parameter
  * EDAC/ti: Fix handling of platform_get_irq() error
  * EDAC/i5100: Fix error handling order in i5100_init_one()
  * crypto: algif_aead - Do not set MAY_BACKLOG on the async path
  * ima: Don't ignore errors from crypto_shash_update()
  * KVM: SVM: Initialize prev_ga_tag before use
  * KVM: x86/mmu: Commit zap of remaining invalid pages when recovering lpages
  * cifs: Return the error from crypt_message when enc/dec key not found.
  * cifs: remove bogus debug code
  * ALSA: hda/realtek: Enable audio jacks of ASUS D700SA with ALC887
  * icmp: randomize the global rate limiter
  * r8169: fix operation under forced interrupt threading
  * tcp: fix to update snd_wl1 in bulk receiver fast path
  * nfc: Ensure presence of NFC_ATTR_FIRMWARE_NAME attribute in nfc_genl_fw_download()
  * net/sched: act_tunnel_key: fix OOB write in case of IPv6 ERSPAN tunnels
  * net: hdlc_raw_eth: Clear the IFF_TX_SKB_SHARING flag after calling ether_setup
  * net: hdlc: In hdlc_rcv, check to make sure dev is an HDLC device
  * chelsio/chtls: correct function return and return type
  * chelsio/chtls: correct netdevice for vlan interface
  * chelsio/chtls: fix socket lock
  * ALSA: bebob: potential info leak in hwdep_read()
  * binder: fix UAF when releasing todo list
  * net/tls: sendfile fails with ktls offload
  * r8169: fix data corruption issue on RTL8402
  * net/ipv4: always honour route mtu during forwarding
  * tipc: fix the skb_unshare() in tipc_buf_append()
  * net: usb: qmi_wwan: add Cellient MPL200 card
  * net/smc: fix valid DMBE buffer sizes
  * net: fix pos incrementment in ipv6_route_seq_next
  * net: fec: Fix PHY init after phy_reset_after_clk_enable()
  * net: fec: Fix phy_device lookup for phy_reset_after_clk_enable()
  * mlx4: handle non-napi callers to napi_poll
  * ipv4: Restore flowi4_oif update before call to xfrm_lookup_route
  * ibmveth: Identify ingress large send packets.
  * ibmveth: Switch order of ibmveth_helper calls.
  * qbt_handler: Memset userspace struct to zero
  * drivers: soc: qcom: Port bam dmux driver
  * drivers: soc: qcom: Add bam dmux driver support
  * msm: kgsl: Compare pid pointer instead of TGID for a new process
  * power: qpnp-smb5: Update legacy cable detection logic in bootup
  * Merge "spi: spi_qsd: Add Shared EE property check for spi"
  * Merge "icnss2: Send power save enter/exit via SMP2P"
  * Merge android-4.19-stable.152 (13abe23) into msm-4.19
  * spi: spi_qsd: Add Shared EE property check for spi
  * soc: qcom: Correct the module description for llcc-orchid
  * soc: qcom: Add LLCC driver for Orchid
  * usb: dwc3: core: Add ipc logs when sg lists are used
  * icnss2: Send power save enter/exit via SMP2P
  * ANDROID: clang: update to 11.0.5
  * FROMLIST: arm64: link with -z norelro regardless of CONFIG_RELOCATABLE
  * defconfig: Enable CONFIG_LEGACY_ENERGY_MODEL_DT for sdm660
  * Merge "msm: pcie: update with link power on check for user PCIe resume"
  * Merge "icnss2: Avoid race between SOC WAKE REQ/RESP"
  * scsi: ufs: Increase fDeviceInit flag poll time to 5sec
  * msm: pcie: update with link power on check for user PCIe resume
  * Merge "defconfig: arm64: msm: enable CONFIG_FORCE_ALLOC_FROM_DMA_ZONE"
  * Merge "msm: ipa: Add ipa rm support for ipa_v2"
  * scsi: ufs: Fix ufshcd_hold dead loop issue if error recovery is handing
  * defconfig: arm64: msm: enable CONFIG_FORCE_ALLOC_FROM_DMA_ZONE
  * ANDROID: GKI: enable CONFIG_WIREGUARD
  * UPSTREAM: wireguard: peerlookup: take lock before checking hash in replace operation
  * UPSTREAM: wireguard: noise: take lock when removing handshake entry from table
  * UPSTREAM: wireguard: queueing: make use of ip_tunnel_parse_protocol
  * UPSTREAM: net: ip_tunnel: add header_ops for layer 3 devices
  * UPSTREAM: wireguard: receive: account for napi_gro_receive never returning GRO_DROP
  * UPSTREAM: wireguard: device: avoid circular netns references
  * UPSTREAM: wireguard: noise: do not assign initiation time in if condition
  * UPSTREAM: wireguard: noise: separate receive counter from send counter
  * UPSTREAM: wireguard: queueing: preserve flow hash across packet scrubbing
  * UPSTREAM: wireguard: noise: read preshared key while taking lock
  * UPSTREAM: wireguard: selftests: use newer iproute2 for gcc-10
  * UPSTREAM: wireguard: send/receive: use explicit unlikely branch instead of implicit coalescing
  * UPSTREAM: wireguard: selftests: initalize ipv6 members to NULL to squelch clang warning
  * UPSTREAM: wireguard: send/receive: cond_resched() when processing worker ringbuffers
  * UPSTREAM: wireguard: socket: remove errant restriction on looping to self
  * UPSTREAM: wireguard: selftests: use normal kernel stack size on ppc64
  * UPSTREAM: wireguard: receive: use tunnel helpers for decapsulating ECN markings
  * UPSTREAM: wireguard: queueing: cleanup ptr_ring in error path of packet_queue_init
  * UPSTREAM: wireguard: send: remove errant newline from packet_encrypt_worker
  * UPSTREAM: wireguard: noise: error out precomputed DH during handshake rather than config
  * UPSTREAM: wireguard: receive: remove dead code from default packet type case
  * UPSTREAM: wireguard: queueing: account for skb->protocol==0
  * UPSTREAM: wireguard: selftests: remove duplicated include <sys/types.h>
  * UPSTREAM: wireguard: socket: remove extra call to synchronize_net
  * UPSTREAM: wireguard: send: account for mtu=0 devices
  * UPSTREAM: wireguard: receive: reset last_under_load to zero
  * UPSTREAM: wireguard: selftests: reduce complexity and fix make races
  * UPSTREAM: wireguard: device: use icmp_ndo_send helper
  * UPSTREAM: wireguard: selftests: tie socket waiting to target pid
  * UPSTREAM: wireguard: selftests: ensure non-addition of peers with failed precomputation
  * UPSTREAM: wireguard: noise: reject peers with low order public keys
  * UPSTREAM: wireguard: allowedips: fix use-after-free in root_remove_peer_lists
  * UPSTREAM: net: skbuff: disambiguate argument and member for skb_list_walk_safe helper
  * UPSTREAM: net: introduce skb_list_walk_safe for skb segment walking
  * UPSTREAM: wireguard: socket: mark skbs as not on list when receiving via gro
  * UPSTREAM: wireguard: queueing: do not account for pfmemalloc when clearing skb header
  * UPSTREAM: wireguard: selftests: remove ancient kernel compatibility code
  * UPSTREAM: wireguard: allowedips: use kfree_rcu() instead of call_rcu()
  * UPSTREAM: wireguard: main: remove unused include <linux/version.h>
  * UPSTREAM: wireguard: global: fix spelling mistakes in comments
  * UPSTREAM: wireguard: Kconfig: select parent dependency for crypto
  * UPSTREAM: wireguard: selftests: import harness makefile for test suite
  * UPSTREAM: net: WireGuard secure network tunnel
  * UPSTREAM: timekeeping: Boot should be boottime for coarse ns accessor
  * UPSTREAM: timekeeping: Add missing _ns functions for coarse accessors
  * UPSTREAM: icmp: introduce helper for nat'd source address in network device context
  * UPSTREAM: crypto: poly1305-x86_64 - Use XORL r32,32
  * UPSTREAM: crypto: curve25519-x86_64 - Use XORL r32,32
  * UPSTREAM: crypto: arm/poly1305 - Add prototype for poly1305_blocks_neon
  * UPSTREAM: crypto: arm/curve25519 - include <linux/scatterlist.h>
  * UPSTREAM: crypto: x86/curve25519 - Remove unused carry variables
  * UPSTREAM: crypto: x86/chacha-sse3 - use unaligned loads for state array
  * UPSTREAM: crypto: lib/chacha20poly1305 - Add missing function declaration
  * UPSTREAM: crypto: arch/lib - limit simd usage to 4k chunks
  * UPSTREAM: crypto: arm[64]/poly1305 - add artifact to .gitignore files
  * UPSTREAM: crypto: x86/curve25519 - leave r12 as spare register
  * UPSTREAM: crypto: x86/curve25519 - replace with formally verified implementation
  * UPSTREAM: crypto: arm64/chacha - correctly walk through blocks
  * UPSTREAM: crypto: x86/curve25519 - support assemblers with no adx support
  * UPSTREAM: crypto: chacha20poly1305 - prevent integer overflow on large input
  * UPSTREAM: crypto: Kconfig - allow tests to be disabled when manager is disabled
  * UPSTREAM: crypto: arm/chacha - fix build failured when kernel mode NEON is disabled
  * UPSTREAM: crypto: x86/poly1305 - emit does base conversion itself
  * UPSTREAM: crypto: chacha20poly1305 - add back missing test vectors and test chunking
  * UPSTREAM: crypto: x86/poly1305 - fix .gitignore typo
  * UPSTREAM: crypto: curve25519 - Fix selftest build error
  * UPSTREAM: crypto: {arm,arm64,mips}/poly1305 - remove redundant non-reduction from emit
  * UPSTREAM: crypto: x86/poly1305 - wire up faster implementations for kernel
  * UPSTREAM: crypto: x86/poly1305 - import unmodified cryptogams implementation
  * UPSTREAM: crypto: poly1305 - add new 32 and 64-bit generic versions
  * UPSTREAM: crypto: lib/curve25519 - re-add selftests
  * UPSTREAM: crypto: arm/curve25519 - add arch-specific key generation function
  * UPSTREAM: crypto: chacha - fix warning message in header file
  * UPSTREAM: crypto: arch - conditionalize crypto api in arch glue for lib code
  * Merge "defcong: sdm660 : Adding support to IPA driver"
  * UPSTREAM: crypto: lib/chacha20poly1305 - use chacha20_crypt()
  * UPSTREAM: crypto: x86/chacha - only unregister algorithms if registered
  * UPSTREAM: crypto: chacha_generic - remove unnecessary setkey() functions
  * UPSTREAM: crypto: lib/chacha20poly1305 - reimplement crypt_from_sg() routine
  * UPSTREAM: crypto: chacha20poly1305 - import construction and selftest from Zinc
  * UPSTREAM: crypto: arm/curve25519 - wire up NEON implementation
  * UPSTREAM: crypto: arm/curve25519 - import Bernstein and Schwabe's Curve25519 ARM implementation
  * UPSTREAM: crypto: curve25519 - x86_64 library and KPP implementations
  * UPSTREAM: crypto: lib/curve25519 - work around Clang stack spilling issue
  * UPSTREAM: crypto: curve25519 - implement generic KPP driver
  * UPSTREAM: crypto: curve25519 - add kpp selftest
  * UPSTREAM: crypto: curve25519 - generic C library implementations
  * UPSTREAM: crypto: blake2s - x86_64 SIMD implementation
  * UPSTREAM: crypto: blake2s - implement generic shash driver
  * UPSTREAM: crypto: testmgr - add test cases for Blake2s
  * UPSTREAM: crypto: blake2s - generic C library implementation and selftest
  * UPSTREAM: crypto: mips/poly1305 - incorporate OpenSSL/CRYPTOGAMS optimized implementation
  * UPSTREAM: crypto: arm/poly1305 - incorporate OpenSSL/CRYPTOGAMS NEON implementation
  * i3c: i3c-master-qcom-geni: Handle timeout for DMA FSM reset
  * UPSTREAM: crypto: arm64/poly1305 - incorporate OpenSSL/CRYPTOGAMS NEON implementation
  * i3c: i3c-master-qcom-geni: Save master device info to debug list
  * UPSTREAM: crypto: x86/poly1305 - expose existing driver as poly1305 library
  * UPSTREAM: crypto: x86/poly1305 - depend on generic library not generic shash
  * UPSTREAM: crypto: poly1305 - expose init/update/final library interface
  * UPSTREAM: crypto: x86/poly1305 - unify Poly1305 state struct with generic code
  * UPSTREAM: crypto: poly1305 - move core routines into a separate library
  * UPSTREAM: crypto: chacha - unexport chacha_generic routines
  * UPSTREAM: crypto: mips/chacha - wire up accelerated 32r2 code from Zinc
  * UPSTREAM: crypto: mips/chacha - import 32r2 ChaCha code from Zinc
  * UPSTREAM: crypto: arm/chacha - expose ARM ChaCha routine as library function
  * UPSTREAM: crypto: arm/chacha - remove dependency on generic ChaCha driver
  * UPSTREAM: crypto: arm/chacha - import Eric Biggers's scalar accelerated ChaCha code
  * UPSTREAM: crypto: arm64/chacha - expose arm64 ChaCha routine as library function
  * UPSTREAM: crypto: arm64/chacha - depend on generic chacha library instead of crypto driver
  * UPSTREAM: crypto: arm64/chacha - use combined SIMD/ALU routine for more speed
  * UPSTREAM: crypto: arm64/chacha - optimize for arbitrary length inputs
  * UPSTREAM: crypto: x86/chacha - expose SIMD ChaCha routine as library function
  * UPSTREAM: crypto: x86/chacha - depend on generic chacha library instead of crypto driver
  * UPSTREAM: crypto: chacha - move existing library code into lib/crypto
  * UPSTREAM: crypto: lib - tidy up lib/crypto Kconfig and Makefile
  * UPSTREAM: crypto: chacha - constify ctx and iv arguments
  * UPSTREAM: crypto: x86/poly1305 - Clear key material from stack in SSE2 variant
  * i3c: i3c-master-qcom-geni: Log error if DMA mode fails
  * defcong: sdm660 : Adding support to IPA driver
  * msm: ipa: Add ipa rm support for ipa_v2
  * UPSTREAM: crypto: xchacha20 - fix comments for test vectors
  * UPSTREAM: crypto: xchacha - add test vector from XChaCha20 draft RFC
  * UPSTREAM: crypto: arm64/chacha - add XChaCha12 support
  * UPSTREAM: crypto: arm64/chacha20 - refactor to allow varying number of rounds
  * UPSTREAM: crypto: arm64/chacha20 - add XChaCha20 support
  * UPSTREAM: crypto: x86/chacha - avoid sleeping under kernel_fpu_begin()
  * UPSTREAM: crypto: x86/chacha - yield the FPU occasionally
  * UPSTREAM: crypto: x86/chacha - add XChaCha12 support
  * UPSTREAM: crypto: x86/chacha20 - refactor to allow varying number of rounds
  * UPSTREAM: crypto: x86/chacha20 - add XChaCha20 support
  * UPSTREAM: crypto: x86/chacha20 - Add a 4-block AVX-512VL variant
  * UPSTREAM: crypto: x86/chacha20 - Add a 2-block AVX-512VL variant
  * UPSTREAM: crypto: x86/chacha20 - Add a 8-block AVX-512VL variant
  * UPSTREAM: crypto: x86/chacha20 - Add a 4-block AVX2 variant
  * UPSTREAM: crypto: x86/chacha20 - Add a 2-block AVX2 variant
  * UPSTREAM: crypto: x86/chacha20 - Use larger block functions more aggressively
  * UPSTREAM: crypto: x86/chacha20 - Support partial lengths in 8-block AVX2 variant
  * UPSTREAM: crypto: x86/chacha20 - Support partial lengths in 4-block SSSE3 variant
  * UPSTREAM: crypto: x86/chacha20 - Support partial lengths in 1-block SSSE3 variant
  * msm: ipa3: Add low-level IPA client support
  * Merge "net: support __alloc_skb to always use GFP_DMA"
  * Merge "msm: ipa2: Add changes compatible to kernel-4.19"
  * Merge "msm: ipa2: Add change to fix ipa padding"
  * Merge "scsi: ufs: reomove Rst_N pulling up action in ufshcd_resume()"
  * Merge "soc: qcom: Add LLCC driver for Orchid"
  * Merge "i3c: i3c-master-qcom-geni: Manage probe time resources"
  * Merge "i3c: i3c-master-qcom-geni: Fix IBI and Hot join related issues"
  * Merge "i3c: i3c-master-qcom-geni: Fix DMA and FIFO mode timeout scenario"
  * net: support __alloc_skb to always use GFP_DMA
  * msm: ipa2: Add changes compatible to kernel-4.19
  * msm: ipa2: Add change to fix ipa padding
  * Merge "msm: ipa: Add support of IPA2 driver"
  * msm: ipa: Add Kconfig changes of IPA2 driver
  * msm: ipa2: Add changes compatible to kernel-4.14
  * i3c: i3c-master-qcom-geni: Fix DMA and FIFO mode timeout scenario
  * ANDROID: GKI: Enable CONFIG_USB_ANNOUNCE_NEW_DEVICES
  * Merge "scsi: ufs: Add back a missing sanity check to ufshcd_read_desc_param()"
  * Merge "fbdev: msm: disable cpu sync during dma_buf_map_attachment"
  * Merge "icnss: check SKIP_QMI test bit for exported qmi messages"
  * Merge "msm: ipa3: Add change to not reset HOLB timer"
  * Merge "msm: kgsl: Don't allow re-importing memory owned by KGSL"
  * msm: ipa: Add support of IPA2 driver
  * scsi: ufs: reomove Rst_N pulling up action in ufshcd_resume()
  * serial: msm_geni_serial: Add new UART IPC log file in DMA mode
  * serial: msm_geni_serial: Add delay for rx invalid transfer
  * msm_geni_serial: Add ioctl for adding new IPC log in uart
  * msm: kgsl: Don't allow re-importing memory owned by KGSL
  * serial: msm_geni_serial: memset RX buffer to Zero
  * Merge android-4.19-stable.149 (9ce79d9) into msm-4.19
  * icnss2: Avoid race between SOC WAKE REQ/RESP
  * ANDROID: GKI: Enable CONFIG_X86_X2APIC
  * i3c: i3c-master-qcom-geni: Manage probe time resources
  * fbdev: msm: disable cpu sync during dma_buf_map_attachment
  * Merge a4cc9369eeea783e3daaaf91b950caf4c10b3c20 on remote branch
  * scsi: ufs: Add back a missing sanity check to ufshcd_read_desc_param()
  * icnss: check SKIP_QMI test bit for exported qmi messages
  * Merge "msm: cvp: Avoid releasing non-existent ARP buffer"
  * Merge "sched/tune: Fix improper accounting of tasks"
  * ANDROID: move builds to use gas prebuilts
  * i3c: i3c-master-qcom-geni: Force the xfer mode as DMA mode
  * Merge "usb: dwc3: Stop active transfer on control endpoints"
  * msm: ipa3: Fix to use proper clock timer
  * Merge "soc: qcom: Fix memcpy operations in ramdump_read"
  * i3c: i3c-master-qcom-geni: Fix IBI and Hot join related issues
  * msm: ipa3: Add change to not reset HOLB timer
  * UPSTREAM: binder: fix UAF when releasing todo list
  * Merge 4.19.152 into android-4.19-stable
  * Linux 4.19.152
  * crypto: qat - check cipher length for aead AES-CBC-HMAC-SHA
  * crypto: bcm - Verify GCM/CCM key length in setkey
  * drivers/net/ethernet/marvell/mvmdio.c: Fix non OF case
  * reiserfs: Fix oops during mount
  * reiserfs: Initialize inode keys properly
  * USB: serial: ftdi_sio: add support for FreeCalypso JTAG+UART adapters
  * USB: serial: pl2303: add device-id for HP GC device
  * staging: comedi: check validity of wMaxPacketSize of usb endpoints found
  * USB: serial: option: Add Telit FT980-KS composition
  * USB: serial: option: add Cellient MPL200 card
  * media: usbtv: Fix refcounting mixup
  * Bluetooth: Disconnect if E0 is used for Level 4
  * Bluetooth: Fix update of connection state in `hci_encrypt_cfm`
  * Bluetooth: Consolidate encryption handling in hci_encrypt_cfm
  * Bluetooth: MGMT: Fix not checking if BT_HS is enabled
  * Bluetooth: L2CAP: Fix calling sk_filter on non-socket based channel
  * Bluetooth: A2MP: Fix not initializing all members
  * ARM: 8867/1: vdso: pass --be8 to linker if necessary
  * ARM: 8939/1: kbuild: use correct nm executable
  * ARM: 8858/1: vdso: use $(LD) instead of $(CC) to link VDSO
  * perf cs-etm: Move definition of 'traceid_list' global variable from header file
  * Merge "Merge android-4.19-stable.146 (443485d) into msm-4.19"
  * Merge android-4.19-stable.146 (443485d) into msm-4.19
  * Merge "cnss2: Add support for PCIE gen switch"
  * Merge "Merge android-4.19-stable.136 (204dd19) into msm-4.19"
  * cnss2: Add support for PCIE gen switch
  * Merge "defconfig: kona: Enable dm-crypt driver"
  * f2fs: code cleanup by removing unnecessary check
  * f2fs: wait for sysfs kobject removal before freeing f2fs_sb_info
  * f2fs: fix writecount false positive in releasing compress blocks
  * f2fs: introduce check_swap_activate_fast()
  * f2fs: don't issue flush in f2fs_flush_device_cache() for nobarrier case
  * f2fs: handle errors of f2fs_get_meta_page_nofail
  * FROMLIST: arm64: vdso32: Allow ld.lld to properly link the VDSO
  * Merge "msm: ipa3: Send enable force clear only for producer pipe"
  * Merge "mmc: core: Fix clk scaling when card max freq below 50MHz"
  * Merge android-4.19-stable.136 (204dd19) into msm-4.19
  * mmc: core: Fix clk scaling when card max freq below 50MHz
  * Merge "mmc: host: Update HS400 timing mode before performing tuning"
  * Merge 4.19.151 into android-4.19-stable
  * Merge "defconfig: msm: Enable FS related configs for Android R"
  * msm: ipa3: Send enable force clear only for producer pipe
  * Linux 4.19.151
  * net: usb: rtl8150: set random MAC address when set_ethernet_addr() fails
  * mm: khugepaged: recalculate min_free_kbytes after memory hotplug as expected by khugepaged
  * mmc: core: don't set limits.discard_granularity as 0
  * perf: Fix task_function_call() error handling
  * rxrpc: Fix server keyring leak
  * rxrpc: Fix some missing _bh annotations on locking conn->state_lock
  * rxrpc: Downgrade the BUG() for unsupported token type in rxrpc_read()
  * rxrpc: Fix rxkad token xdr encoding
  * net/mlx5e: Fix VLAN create flow
  * net/mlx5e: Fix VLAN cleanup flow
  * net: usb: ax88179_178a: fix missing stop entry in driver_info
  * mdio: fix mdio-thunder.c dependency & build error
  * bonding: set dev->needed_headroom in bond_setup_by_slave()
  * xfrm: Use correct address family in xfrm_state_find
  * platform/x86: fix kconfig dependency warning for FUJITSU_LAPTOP
  * net: stmmac: removed enabling eee in EEE set callback
  * xfrm: clone whole liftime_cur structure in xfrm_do_migrate
  * xfrm: clone XFRMA_SEC_CTX in xfrm_do_migrate
  * xfrm: clone XFRMA_REPLAY_ESN_VAL in xfrm_do_migrate
  * xfrm: clone XFRMA_SET_MARK in xfrm_do_migrate
  * drm/amdgpu: prevent double kfree ttm->sg
  * openvswitch: handle DNAT tuple collision
  * net: team: fix memory leak in __team_options_register
  * team: set dev->needed_headroom in team_setup_by_port()
  * sctp: fix sctp_auth_init_hmacs() error path
  * i2c: owl: Clear NACK and BUS error bits
  * i2c: meson: fixup rate calculation with filter delay
  * i2c: meson: fix clock setting overwrite
  * cifs: Fix incomplete memory allocation on setxattr path
  * xfrmi: drop ignore_df check before updating pmtu
  * mm/khugepaged: fix filemap page_to_pgoff(page) != offset
  * macsec: avoid use-after-free in macsec_handle_frame()
  * nvme-core: put ctrl ref when module ref get fail
  * arm64: dts: stratix10: add status to qspi dts node
  * mtd: rawnand: sunxi: Fix the probe error path
  * i2c: i801: Exclude device from suspend direct complete optimization
  * perf top: Fix stdio interface input handling with glibc 2.28+
  * driver core: Fix probe_count imbalance in really_probe()
  * platform/x86: thinkpad_acpi: re-initialize ACPI buffer size when reuse
  * platform/x86: intel-vbtn: Switch to an allow-list for SW_TABLET_MODE reporting
  * platform/x86: thinkpad_acpi: initialize tp_nvram_state variable
  * platform/x86: intel-vbtn: Fix SW_TABLET_MODE always reporting 1 on the HP Pavilion 11 x360
  * usermodehelper: reset umask to default before executing user process
  * drm/nouveau/mem: guard against NULL pointer access in mem_del
  * net: wireless: nl80211: fix out-of-bounds access in nl80211_del_key()
  * Revert "ravb: Fixed to be able to unload modules"
  * fbcon: Fix global-out-of-bounds read in fbcon_get_font()
  * Fonts: Support FONT_EXTRA_WORDS macros for built-in fonts
  * fbdev, newport_con: Move FONT_EXTRA_WORDS macros into linux/font.h
  * sched/tune: Fix improper accounting of tasks
  * Merge "msm: kgsl: Remove dev_err() from fenced write loop"
  * Merge "net_sched: Add flow control support to prio qdisc"
  * Merge "uapi: sound: Fix compilation error"
  * Merge "Add support for block disk encryption"
  * usb: dwc3: Stop active transfer on control endpoints
  * Add support for block disk encryption
  * mmc: host: Use right parameter for ext4 plus eMMC
  * mmc: host: reprogram the key to cover the invalid config case
  * Fix OTA issue with vts fixes for new fbe framework
  * Merge "firmware: qcom: Fix hyp_log issue"
  * msm: kgsl: Remove dev_err() from fenced write loop
  * uapi: sound: Fix compilation error
  * Merge "clk: qcom: mdss: Update dfps data struct for FB targets"
  * defconfig: msm: Enable FS related configs for Android R
  * Merge "net:sockev: hold file reference till the sock event is sent"
  * net:sockev: hold file reference till the sock event is sent
  * net_sched: Add flow control support to prio qdisc
  * msm: camera: Fix for memory leak
  * defconfig: kona: Enable dm-crypt driver
  * Merge "md: dm-default-key: Use system sector size for SDHCI devices"
  * soc: qcom: Fix memcpy operations in ramdump_read
  * md: dm-default-key: Use system sector size for SDHCI devices
  * Merge "defconfig: msm: Enable CONFIG_STATIC_USERMODEHELPER for bengal_32"
  * Merge "fdt: Update CRC check for rng-seed"
  * defconfig: msm: Enable CONFIG_STATIC_USERMODEHELPER for bengal_32
  * Merge "fbdev: msm: remove mappings before iommu detach"
  * fdt: Update CRC check for rng-seed
  * Merge "msm: kgsl: Use regulator_is_enabled api when gpu-quirk-cx-gdsc is defined"
  * usb: phy: snps: Enable auto resume feature only in host mode
  * f2fs: fix to set SBI_NEED_FSCK flag for inconsistent inode
  * msm: ipa: Fix rndis client disconnection gracefully
  * Merge "icnss2: Decrement soc wake ref count"
  * Merge "binder: update low_latency selection for binder transactions"
  * f2fs: reject CASEFOLD inode flag without casefold feature
  * f2fs: fix memory alignment to support 32bit
  * Merge "icnss2: Send enter power save after driver suspend"
  * Merge "cnss_utils: Update wlfw power save enter/exit qmi message"
  * Merge "power: smb5: Fix LPD flag for PMI632"
  * Merge "icnss: reject idle restart if wlan driver unregistered"
  * Merge "NFC: Add support for core init command"
  * icnss: reject idle restart if wlan driver unregistered
  * soc: qcom: Add LLCC driver for Orchid
  * icnss2: Decrement soc wake ref count
  * Merge 4.19.150 into android-4.19-stable
  * Linux 4.19.150
  * netfilter: ctnetlink: add a range check for l3/l4 protonum
  * ep_create_wakeup_source(): dentry name can change under you...
  * epoll: EPOLL_CTL_ADD: close the race in decision to take fast path
  * epoll: replace ->visited/visited_list with generation count
  * epoll: do not insert into poll queues until all sanity checks are done
  * net/packet: fix overflow in tpacket_rcv
  * mm: don't rely on system state to detect hot-plug operations
  * mm: replace memmap_context by meminit_context
  * random32: Restore __latent_entropy attribute on net_rand_state
  * Input: trackpoint - enable Synaptics trackpoints
  * i2c: cpm: Fix i2c_ram structure
  * iommu/exynos: add missing put_device() call in exynos_iommu_of_xlate()
  * clk: samsung: exynos4: mark 'chipid' clock as CLK_IGNORE_UNUSED
  * nfs: Fix security label length not being reset
  * pinctrl: mvebu: Fix i2c sda definition for 98DX3236
  * gpio: sprd: Clear interrupt when setting the type as edge
  * nvme-fc: fail new connections to a deleted host or remote port
  * spi: fsl-espi: Only process interrupts for expected events
  * mac80211: do not allow bigger VHT MPDUs than the hardware supports
  * drivers/net/wan/hdlc: Set skb->protocol before transmitting
  * drivers/net/wan/lapbether: Make skb->protocol consistent with the header
  * nvme-core: get/put ctrl and transport module in nvme_dev_open/release()
  * rndis_host: increase sleep time in the query-response loop
  * net: dec: de2104x: Increase receive ring size for Tulip
  * drm/sun4i: mixer: Extend regmap max_register
  * drivers/net/wan/hdlc_fr: Add needed_headroom for PVC devices
  * drm/amdgpu: restore proper ref count in amdgpu_display_crtc_set_config
  * ftrace: Move RCU is watching check after recursion check
  * Input: i8042 - add nopnp quirk for Acer Aspire 5 A515
  * net: virtio_vsock: Enhance connection semantics
  * vsock/virtio: add transport parameter to the virtio_transport_reset_no_sock()
  * vsock/virtio: stop workers during the .remove()
  * vsock/virtio: use RCU to avoid use-after-free on the_virtio_vsock
  * clk: socfpga: stratix10: fix the divider for the emac_ptp_free_clk
  * gpio: tc35894: fix up tc35894 interrupt configuration
  * gpio: mockup: fix resource leak in error path
  * USB: gadget: f_ncm: Fix NDP16 datagram validation
  * mmc: sdhci: Workaround broken command queuing on Intel GLK based IRBIS models
  * fbdev: msm: remove mappings before iommu detach
  * clk: qcom: mdss: Update dfps data struct for FB targets
  * usb: pd: Register typec partner in case AAA is connected
  * binder: update low_latency selection for binder transactions
  * sched/walt: Improve the scheduler
  * msm: kgsl: Use regulator_is_enabled api when gpu-quirk-cx-gdsc is defined
  * Merge "defconfig: Enable CONFIG_UTS_NS for sdm660"
  * icnss2: Send enter power save after driver suspend
  * mmc: host: Update HS400 timing mode before performing tuning
  * Merge "icnss2: Fix race condition during SOC wake req/release"
  * Merge "icnss2: Avoid calibration during SSR of WCN6750"
  * Merge "trace: f2fs: Fix kasan slab-out-of-bounds"
  * Merge "msm: kgsl: Don't wait for room in context queue when context is invalidated"
  * Merge "usb: core: Don't wait for completion of urbs"
  * Merge "soc: qcom: Fix smcinvoke_obj->fd assignment"
  * defconfig: Enable CONFIG_UTS_NS for sdm660
  * Merge e3105f1e5e6b9b02a52bf9f8093f07a544588c75 on remote branch
  * Merge "msm: kgsl: Update clk_set_rate() sequence for bimc_gpu_clk"
  * trace: f2fs: Fix kasan slab-out-of-bounds
  * Merge "drm: increase max limit of drm open count to 128"
  * ANDROID: use arm-linux-androidkernel- for CROSS_COMPILE_COMPAT
  * ANDROID: build.config.common: enable LLVM=1
  * Merge 4.19.149 into android-4.19-stable
  * Linux 4.19.149
  * KVM: arm64: Assume write fault on S1PTW permission fault on instruction fetch
  * ata: sata_mv, avoid trigerrable BUG_ON
  * ata: make qc_prep return ata_completion_errors
  * ata: define AC_ERR_OK
  * kprobes: Fix compiler warning for !CONFIG_KPROBES_ON_FTRACE
  * s390/zcrypt: Fix ZCRYPT_PERDEV_REQCNT ioctl
  * mm, THP, swap: fix allocating cluster for swapfile by mistake
  * kprobes: Fix to check probe enabled before disarm_kprobe_ftrace()
  * s390/dasd: Fix zero write for FBA devices
  * tracing: fix double free
  * KVM: SVM: Add a dedicated INVD intercept routine
  * KVM: x86: Reset MMU context if guest toggles CR4.SMAP or CR4.PKE
  * MIPS: Add the missing 'CPU_1074K' into __get_cpu_type()
  * regmap: fix page selection for noinc reads
  * ALSA: asihpi: fix iounmap in error handler
  * bpf: Fix a rcu warning for bpffs map pretty-print
  * batman-adv: mcast: fix duplicate mcast packets from BLA backbone to mesh
  * batman-adv: mcast: fix duplicate mcast packets in BLA backbone from mesh
  * batman-adv: Add missing include for in_interrupt()
  * drm/sun4i: sun8i-csc: Secondary CSC register correction
  * net: qed: RDMA personality shouldn't fail VF load
  * drm/vc4/vc4_hdmi: fill ASoC card owner
  * bpf: Fix clobbering of r2 in bpf_gen_ld_abs
  * mac802154: tx: fix use-after-free
  * batman-adv: mcast/TT: fix wrongly dropped or rerouted packets
  * atm: eni: fix the missed pci_disable_device() for eni_init_one()
  * batman-adv: bla: fix type misuse for backbone_gw hash indexing
  * mwifiex: Increase AES key storage size to 256 bits
  * clocksource/drivers/h8300_timer8: Fix wrong return value in h8300_8timer_init()
  * ieee802154/adf7242: check status of adf7242_read_reg
  * ieee802154: fix one possible memleak in ca8210_dev_com_init
  * objtool: Fix noreturn detection for ignored functions
  * i2c: core: Call i2c_acpi_install_space_handler() before i2c_acpi_register_devices()
  * drm/amdkfd: fix a memory leak issue
  * lockdep: fix order in trace_hardirqs_off_caller()
  * s390/init: add missing __init annotations
  * RISC-V: Take text_mutex in ftrace_init_nop()
  * ASoC: Intel: bytcr_rt5640: Add quirk for MPMAN Converter9 2-in-1
  * ASoC: wm8994: Ensure the device is resumed in wm89xx_mic_detect functions
  * ASoC: wm8994: Skip setting of the WM8994_MICBIAS register for WM1811
  * nvme: explicitly update mpath disk capacity on revalidation
  * net: openvswitch: use div_u64() for 64-by-32 divisions
  * perf parse-events: Use strcmp() to compare the PMU name
  * ubi: fastmap: Free unused fastmap anchor peb during detach
  * btrfs: qgroup: fix data leak caused by race between writeback and truncate
  * vfio/pci: fix racy on error and request eventfd ctx
  * selftests/x86/syscall_nt: Clear weird flags after each test
  * scsi: libfc: Skip additional kref updating work event
  * scsi: libfc: Handling of extra kref
  * nvme: fix possible deadlock when I/O is blocked
  * cifs: Fix double add page to memcg when cifs_readpages
  * vfio/pci: Clear error and request eventfd ctx after releasing
  * x86/speculation/mds: Mark mds_user_clear_cpu_buffers() __always_inline
  * mtd: parser: cmdline: Support MTD names containing one or more colons
  * rapidio: avoid data race between file operation callbacks and mport_cdev_add().
  * mm/swap_state: fix a data race in swapin_nr_pages
  * ceph: fix potential race in ceph_check_caps
  * PCI: tegra: Fix runtime PM imbalance on error
  * mtd: rawnand: omap_elm: Fix runtime PM imbalance on error
  * wlcore: fix runtime pm imbalance in wlcore_regdomain_config
  * wlcore: fix runtime pm imbalance in wl1271_tx_work
  * ASoC: img-i2s-out: Fix runtime PM imbalance on error
  * perf kcore_copy: Fix module map when there are no modules loaded
  * perf metricgroup: Free metric_events on error
  * perf util: Fix memory leak of prefix_if_not_in
  * perf stat: Fix duration_time value for higher intervals
  * perf trace: Fix the selection for architectures to generate the errno name tables
  * perf evsel: Fix 2 memory leaks
  * vfio/pci: fix memory leaks of eventfd ctx
  * btrfs: don't force read-only after error in drop snapshot
  * usb: dwc3: Increase timeout for CmdAct cleared by device controller
  * printk: handle blank console arguments passed in.
  * drm/nouveau/dispnv50: fix runtime pm imbalance on error
  * drm/nouveau: fix runtime pm imbalance on error
  * drm/nouveau/debugfs: fix runtime pm imbalance on error
  * e1000: Do not perform reset in reset_task if we are already down
  * arm64/cpufeature: Drop TraceFilt feature exposure from ID_DFR0 register
  * scsi: cxlflash: Fix error return code in cxlflash_probe()
  * USB: EHCI: ehci-mv: fix less than zero comparison of an unsigned int
  * fuse: don't check refcount after stealing page
  * powerpc/traps: Make unrecoverable NMIs die instead of panic
  * ALSA: hda: Fix potential race in unsol event handler
  * tty: serial: samsung: Correct clock selection logic
  * tipc: fix memory leak in service subscripting
  * USB: EHCI: ehci-mv: fix error handling in mv_ehci_probe()
  * Bluetooth: Handle Inquiry Cancel error after Inquiry Complete
  * phy: samsung: s5pv210-usb2: Add delay after reset
  * power: supply: max17040: Correct voltage reading
  * perf mem2node: Avoid double free related to realloc
  * atm: fix a memory leak of vcc->user_back
  * dt-bindings: sound: wm8994: Correct required supplies based on actual implementaion
  * arm64: cpufeature: Relax checks for AArch32 support at EL[0-2]
  * sparc64: vcc: Fix error return code in vcc_probe()
  * staging:r8188eu: avoid skb_clone for amsdu to msdu conversion
  * scsi: aacraid: Fix error handling paths in aac_probe_one()
  * net: openvswitch: use u64 for meter bucket
  * KVM: arm64: vgic-its: Fix memory leak on the error path of vgic_add_lpi()
  * drivers: char: tlclk.c: Avoid data race between init and interrupt handler
  * bdev: Reduce time holding bd_mutex in sync in blkdev_close()
  * KVM: Remove CREATE_IRQCHIP/SET_PIT2 race
  * serial: uartps: Wait for tx_empty in console setup
  * scsi: qedi: Fix termination timeouts in session logout
  * mm/mmap.c: initialize align_offset explicitly for vm_unmapped_area
  * nvmet-rdma: fix double free of rdma queue
  * mm/vmscan.c: fix data races using kswapd_classzone_idx
  * mm/filemap.c: clear page error before actual read
  * mm/kmemleak.c: use address-of operator on section symbols
  * NFS: Fix races nfs_page_group_destroy() vs nfs_destroy_unlinked_subrequests()
  * PCI: pciehp: Fix MSI interrupt race
  * ALSA: usb-audio: Fix case when USB MIDI interface has more than one extra endpoint descriptor
  * ubifs: Fix out-of-bounds memory access caused by abnormal value of node_len
  * PCI: Use ioremap(), not phys_to_virt() for platform ROM
  * svcrdma: Fix leak of transport addresses
  * SUNRPC: Fix a potential buffer overflow in 'svc_print_xprts()'
  * scsi: hpsa: correct race condition in offload enabled
  * RDMA/rxe: Set sys_image_guid to be aligned with HW IB devices
  * nvme: Fix controller creation races with teardown flow
  * nvme-multipath: do not reset on unknown status
  * tools: gpio-hammer: Avoid potential overflow in main
  * cpufreq: powernv: Fix frame-size-overflow in powernv_cpufreq_work_fn
  * perf cpumap: Fix snprintf overflow check
  * serial: 8250: 8250_omap: Terminate DMA before pushing data on RX timeout
  * serial: 8250_omap: Fix sleeping function called from invalid context during probe
  * serial: 8250_port: Don't service RX FIFO if throttled
  * perf parse-events: Fix 3 use after frees found with clang ASAN
  * thermal: rcar_thermal: Handle probe error gracefully
  * tracing: Use address-of operator on section symbols
  * drm/msm/a5xx: Always set an OPP supported hardware value
  * drm/msm: fix leaks if initialization fails
  * KVM: PPC: Book3S HV: Treat TM-related invalid form instructions on P9 like the valid ones
  * RDMA/cm: Remove a race freeing timewait_info
  * nfsd: Don't add locks to closed or closing open stateids
  * rtc: ds1374: fix possible race condition
  * rtc: sa1100: fix possible race condition
  * tpm: ibmvtpm: Wait for buffer to be set before proceeding
  * ext4: mark block bitmap corrupted when found instead of BUGON
  * xfs: mark dir corrupt when lookup-by-hash fails
  * xfs: don't ever return a stale pointer from __xfs_dir3_free_read
  * media: tda10071: fix unsigned sign extension overflow
  * Bluetooth: L2CAP: handle l2cap config request during open state
  * scsi: aacraid: Disabling TM path and only processing IOP reset
  * ath10k: use kzalloc to read for ath10k_sdio_hif_diag_read
  * drm/amd/display: Stop if retimer is not available
  * drm/amdgpu: increase atombios cmd timeout
  * mm: avoid data corruption on CoW fault into PFN-mapped VMA
  * perf jevents: Fix leak of mapfile memory
  * ext4: fix a data race at inode->i_disksize
  * timekeeping: Prevent 32bit truncation in scale64_check_overflow()
  * Bluetooth: guard against controllers sending zero'd events
  * media: go7007: Fix URB type for interrupt handling
  * bus: hisi_lpc: Fixup IO ports addresses to avoid use-after-free in host removal
  * random: fix data races at timer_rand_state
  * firmware: arm_sdei: Use cpus_read_lock() to avoid races with cpuhp
  * drm/amd/display: dal_ddc_i2c_payloads_create can fail causing panic
  * dmaengine: tegra-apb: Prevent race conditions on channel's freeing
  * dmaengine: stm32-dma: use vchan_terminate_vdesc() in .terminate_all
  * bpf: Remove recursion prevention from rcu free callback
  * x86/pkeys: Add check for pkey "overflow"
  * media: staging/imx: Missing assignment in imx_media_capture_device_register()
  * dmaengine: stm32-mdma: use vchan_terminate_vdesc() in .terminate_all
  * KVM: x86: fix incorrect comparison in trace event
  * RDMA/rxe: Fix configuration of atomic queue pair attributes
  * perf test: Fix test trace+probe_vfs_getname.sh on s390
  * ALSA: usb-audio: Don't create a mixer element with bogus volume range
  * mt76: clear skb pointers from rx aggregation reorder buffer during cleanup
  * crypto: chelsio - This fixes the kernel panic which occurs during a libkcapi test
  * clk: stratix10: use do_div() for 64-bit calculation
  * drm/omap: fix possible object reference leak
  * scsi: lpfc: Fix coverity errors in fmdi attribute handling
  * scsi: lpfc: Fix RQ buffer leakage when no IOCBs available
  * selinux: sel_avc_get_stat_idx should increase position index
  * audit: CONFIG_CHANGE don't log internal bookkeeping as an event
  * skbuff: fix a data race in skb_queue_len()
  * ALSA: hda: Clear RIRB status before reading WP
  * KVM: fix overflow of zero page refcount with ksm running
  * Bluetooth: prefetch channel before killing sock
  * mm: pagewalk: fix termination condition in walk_pte_range()
  * mm/swapfile.c: swap_next should increase position index
  * Bluetooth: Fix refcount use-after-free issue
  * tools/power/x86/intel_pstate_tracer: changes for python 3 compatibility
  * selftests/ftrace: fix glob selftest
  * ceph: ensure we have a new cap before continuing in fill_inode
  * ar5523: Add USB ID of SMCWUSBT-G2 wireless adapter
  * ARM: 8948/1: Prevent OOB access in stacktrace
  * tracing: Set kernel_stack's caller size properly
  * Bluetooth: btrtl: Use kvmalloc for FW allocations
  * powerpc/eeh: Only dump stack once if an MMIO loop is detected
  * s390/cpum_sf: Use kzalloc and minor changes
  * dmaengine: zynqmp_dma: fix burst length configuration
  * scsi: ufs: Fix a race condition in the tracing code
  * scsi: ufs: Make ufshcd_add_command_trace() easier to read
  * ACPI: EC: Reference count query handlers under lock
  * sctp: move trace_sctp_probe_path into sctp_outq_sack
  * media: ti-vpe: cal: Restrict DMA to avoid memory corruption
  * seqlock: Require WRITE_ONCE surrounding raw_seqcount_barrier
  * ipv6_route_seq_next should increase position index
  * rt_cpu_seq_next should increase position index
  * neigh_stat_seq_next() should increase position index
  * xfs: fix log reservation overflows when allocating large rt extents
  * KVM: arm/arm64: vgic: Fix potential double free dist->spis in __kvm_vgic_destroy()
  * kernel/sys.c: avoid copying possible padding bytes in copy_to_user
  * ASoC: max98090: remove msleep in PLL unlocked workaround
  * CIFS: Properly process SMB3 lease breaks
  * debugfs: Fix !DEBUG_FS debugfs_create_automount
  * scsi: pm80xx: Cleanup command when a reset times out
  * gfs2: clean up iopen glock mess in gfs2_create_inode
  * mmc: core: Fix size overflow for mmc partitions
  * ubi: Fix producing anchor PEBs
  * RDMA/iw_cgxb4: Fix an error handling path in 'c4iw_connect()'
  * xfs: fix attr leaf header freemap.size underflow
  * fix dget_parent() fastpath race
  * RDMA/i40iw: Fix potential use after free
  * RDMA/qedr: Fix potential use after free
  * dmaengine: mediatek: hsdma_probe: fixed a memory leak when devm_request_irq fails
  * bcache: fix a lost wake-up problem caused by mca_cannibalize_lock
  * tracing: Adding NULL checks for trace_array descriptor pointer
  * tpm_crb: fix fTPM on AMD Zen+ CPUs
  * drm/amdgpu/powerplay/smu7: fix AVFS handling with custom powerplay table
  * mfd: mfd-core: Protect against NULL call-back function pointer
  * mtd: cfi_cmdset_0002: don't free cfi->cfiq in error path of cfi_amdstd_setup()
  * drm/amdgpu/powerplay: fix AVFS handling with custom powerplay table
  * clk/ti/adpll: allocate room for terminating null
  * net: silence data-races on sk_backlog.tail
  * scsi: lpfc: Fix kernel crash at lpfc_nvme_info_show during remote port bounce
  * scsi: fnic: fix use after free
  * PM / devfreq: tegra30: Fix integer overflow on CPU's freq max out
  * leds: mlxreg: Fix possible buffer overflow
  * lib/string.c: implement stpcpy
  * ALSA: hda/realtek: Enable front panel headset LED on Lenovo ThinkStation P520
  * ALSA: hda/realtek - Couldn't detect Mic if booting with headset plugged
  * ALSA: usb-audio: Add delay quirk for H570e USB headsets
  * x86/ioapic: Unbreak check_timer()
  * arch/x86/lib/usercopy_64.c: fix __copy_user_flushcache() cache writeback
  * media: smiapp: Fix error handling at NVM reading
  * ASoC: kirkwood: fix IRQ error handling
  * gma/gma500: fix a memory disclosure bug due to uninitialized bytes
  * m68k: q40: Fix info-leak in rtc_ioctl
  * scsi: aacraid: fix illegal IO beyond last LBA
  * mm: fix double page fault on arm64 if PTE_AF is cleared
  * ath10k: fix memory leak for tpc_stats_final
  * ath10k: fix array out-of-bounds access
  * dma-fence: Serialise signal enabling (dma_fence_enable_sw_signaling)
  * media: mc-device.c: fix memleak in media_device_register_entity
  * selinux: allow labeling before policy is loaded
  * NFC: Add support for core init command
  * Merge "dt-bindings: msm: Add bindings for MP limit fuse support"
  * icnss2: Fix race condition during SOC wake req/release
  * Merge "scsi: ufs: Fix ufs power down/on specs violation in suspend/resume path"
  * Merge "fbdev: msm: set smmu cb domain attached during probe"
  * msm: kgsl: Don't wait for room in context queue when context is invalidated
  * dt-bindings: msm: Add bindings for MP limit fuse support
  * fbdev: msm: set smmu cb domain attached during probe
  * Merge "scsi: ufs: Fix unexpected values get from ufshcd_read_desc_param()"
  * Merge "mm/Kconfig: forcing allocators to return ZONE_DMA32 memory"
  * Merge "defconfig: enable SDE rotator"
  * Merge "edac: improve gold CPU cache way parsing"
  * Merge "usb: dwc3: gadget: Properly handle failed kick_transfer"
  * defconfig: enable SDE rotator
  * f2fs: fix slab leak of rpages pointer
  * f2fs: compress: fix to disallow enabling compress on non-empty file
  * f2fs: compress: introduce cic/dic slab cache
  * f2fs: compress: introduce page array slab cache
  * mm/Kconfig: forcing allocators to return ZONE_DMA32 memory
  * power: smb5: Fix LPD flag for PMI632
  * ANDROID: GKI: prevent removal of monitored symbols
  * firmware: qcom: Fix hyp_log issue
  * f2fs: fix to do sanity check on segment/section count
  * f2fs: fix to check segment boundary during SIT page readahead
  * f2fs: fix uninit-value in f2fs_lookup
  * vfs: track per-sb writeback errors and report them to syncfs
  * f2fs: remove unneeded parameter in find_in_block()
  * f2fs: fix wrong total_sections check and fsmeta check
  * f2fs: remove duplicated code in sanity_check_area_boundary
  * f2fs: remove unused check on version_bitmap
  * f2fs: relocate blkzoned feature check
  * f2fs: do sanity check on zoned block device path
  * f2fs: add trace exit in exception path
  * f2fs: change return value of reserved_segments to unsigned int
  * soc: qcom: Fix smcinvoke_obj->fd assignment
  * drm: increase max limit of drm open count to 128
  * Merge "usb: gadget: Don't giveback request if ep command times out"
  * scsi: ufs: Fix unexpected values get from ufshcd_read_desc_param()
  * Merge "msm: sde: rotator: enable sde rotator"
  * usb: gadget: Don't giveback request if ep command times out
  * icnss2: Avoid calibration during SSR of WCN6750
  * Merge "f2fs: remove blk_plugging in block_operations"
  * Merge "leds: qpnp-flash-v2: Acquire the bms_psy handle at runtime"
  * ANDROID: Refresh ABI.xmls with libabigail 1.8.0-98bbf30d
  * edac: improve gold CPU cache way parsing
  * msm: kgsl: Update clk_set_rate() sequence for bimc_gpu_clk
  * Merge "regulator: qpnp-lcdb: Disable the SC irq only for PM660L V1.1 and below"
  * scsi: ufs: Fix ufs power down/on specs violation in suspend/resume path
  * Merge "mmc: Remove unused code"
  * Merge 4.19.148 into android-4.19-stable
  * Merge "cnss2: Enable self-recovery only when host driver detects linkdown"
  * Linux 4.19.148
  * serial: 8250: Avoid error message on reprobe
  * tcp_bbr: adapt cwnd based on ack aggregation estimation
  * tcp_bbr: refactor bbr_target_cwnd() for general inflight provisioning
  * mm: memcg: fix memcg reclaim soft lockup
  * kbuild: support LLVM=1 to switch the default tools to Clang/LLVM
  * kbuild: replace AS=clang with LLVM_IAS=1
  * kbuild: remove AS variable
  * x86/boot: kbuild: allow readelf executable to be specified
  * net: wan: wanxl: use $(M68KCC) instead of $(M68KAS) for rebuilding firmware
  * net: wan: wanxl: use allow to pass CROSS_COMPILE_M68k for rebuilding firmware
  * Documentation/llvm: fix the name of llvm-size
  * Documentation/llvm: add documentation on building w/ Clang/LLVM
  * kbuild: add OBJSIZE variable for the size tool
  * MAINTAINERS: add CLANG/LLVM BUILD SUPPORT info
  * ipv4: Update exception handling for multipath routes via same device
  * net: add __must_check to skb_put_padto()
  * net: qrtr: check skb_put_padto() return value
  * net: phy: Avoid NPD upon phy_detach() when driver is unbound
  * bnxt_en: Protect bnxt_set_eee() and bnxt_set_pauseparam() with mutex.
  * bnxt_en: return proper error codes in bnxt_show_temp
  * tipc: use skb_unshare() instead in tipc_buf_append()
  * tipc: fix shutdown() of connection oriented socket
  * tipc: Fix memory leak in tipc_group_create_member()
  * nfp: use correct define to return NONE fec
  * net: sch_generic: aviod concurrent reset and enqueue op for lockless qdisc
  * net: ipv6: fix kconfig dependency warning for IPV6_SEG6_HMAC
  * net: dsa: rtl8366: Properly clear member config
  * net: DCB: Validate DCB_ATTR_DCB_BUFFER argument
  * ipv6: avoid lockdep issue in fib6_del()
  * ip: fix tos reflection in ack and reset packets
  * hdlc_ppp: add range checks in ppp_cp_parse_cr()
  * geneve: add transport ports in route lookup for geneve
  * cxgb4: Fix offset when clearing filter byte counters
  * mm/thp: fix __split_huge_pmd_locked() for migration PMD
  * kprobes: fix kill kprobe which has been marked as gone
  * KVM: fix memory leak in kvm_io_bus_unregister_dev()
  * af_key: pfkey_dump needs parameter validation
  * msm: sde: rotator: enable sde rotator
  * Merge "msm: pcie: provide client the ability to control PCIe target_link_speed"
  * Merge "cnss2: Add runtime PM stats support"
  * Merge "cnss2: Log a message after assert/de-assert WLAN_EN GPIO"
  * Merge "msm: mink: Fix copy_to_user issue"
  * msm: pcie: provide client the ability to control PCIe target_link_speed
  * Merge "cnss2: Release qmi handle after server exit"
  * Merge "dcc_v2: fix 1 write 1 read register configuration fail issue"
  * qcom: qpnp-fg-gen3: Continue fg_gen3_probe() when !DEBUG_FS
  * dcc_v2: fix 1 write 1 read register configuration fail issue
  * leds: qpnp-flash-v2: Acquire the bms_psy handle at runtime
  * regulator: qpnp-labibb: Don't handle LAB_VREG_OK in TTW mode for pmi8998
  * regulator: qpnp-labibb: Add sysfs class to enable/disable the irq
  * regulator: qpnp-lcdb: Disable the SC irq only for PM660L V1.1 and below
  * regulator: qpnp-lcdb: Add sysfs class to enable/disable the irq
  * cnss2: Enable self-recovery only when host driver detects linkdown
  * ANDROID: drop KERNEL_DIR setting in build.config.common
  * cnss2: Release qmi handle after server exit
  * UPSTREAM: driver core: Avoid deferred probe due to fw_devlink_pause/resume()
  * UPSTREAM: driver core: Rename dev_links_info.defer_sync to defer_hook
  * UPSTREAM: driver core: Don't do deferred probe in parallel with kernel_init thread
  * Merge "crypto: Fix possible stack out of bound error"
  * msm: cvp: Avoid releasing non-existent ARP buffer
  * dm-crypt: Skip encryption if bio is fscrypto or blk-crypto encrypted
  * msm: mink: Fix copy_to_user issue
  * Merge 4.19.147 into android-4.19-stable
  * Merge "cnss2: Collect shadow registers for RDDM scenario"
  * cnss_utils: Update wlfw power save enter/exit qmi message
  * crypto: Fix possible stack out of bound error
  * Merge "mhi: core: Extend mhi_device_get_sync_atomic() for panic cases"
  * mhi: core: Extend mhi_device_get_sync_atomic() for panic cases
  * Merge "power: smblite-lib: Reduce the ICL immediately when flash is active"
  * Merge "cnss2: Assert when power on retry reaches maximum"
  * cnss2: Log a message after assert/de-assert WLAN_EN GPIO
  * cnss2: Assert when power on retry reaches maximum
  * usb: dwc3: gadget: Properly handle failed kick_transfer
  * Merge "msm: kgsl: Enable process reclaim for A610"
  * Merge "qseecom: Propagate correct return value from TZ"
  * Merge "serial: msm_geni_serial: Don't use WARN_ON for console uart"
  * Merge "Revert "arm64: kpti: force off kpti""
  * Merge "arm: dma-mapping.c: add cpu sync in map_sg and unmap_sg"
  * Merge "defconfig: Enable blk-crypto-fallback to handle sw crypto request"
  * f2fs: remove blk_plugging in block_operations
  * Merge "irqchip: gic-v3: Add support to get pending irqs"
  * Linux 4.19.147
  * x86/defconfig: Enable CONFIG_USB_XHCI_HCD=y
  * powerpc/dma: Fix dma_map_ops::get_required_mask
  * ehci-hcd: Move include to keep CRC stable
  * x86/boot/compressed: Disable relocation relaxation
  * serial: 8250_pci: Add Realtek 816a and 816b
  * Input: i8042 - add Entroware Proteus EL07R4 to nomux and reset lists
  * Input: trackpoint - add new trackpoint variant IDs
  * percpu: fix first chunk size calculation for populated bitmap
  * Revert "ALSA: hda - Fix silent audio output and corrupted input on MSI X570-A PRO"
  * i2c: i801: Fix resume bug
  * usblp: fix race between disconnect() and read()
  * USB: UAS: fix disconnect by unplugging a hub
  * USB: quirks: Add USB_QUIRK_IGNORE_REMOTE_WAKEUP quirk for BYD zhaoxin notebook
  * drm/mediatek: Add missing put_device() call in mtk_hdmi_dt_parse_pdata()
  * drm/mediatek: Add exception handing in mtk_drm_probe() if component init fail
  * MIPS: SNI: Fix spurious interrupts
  * fbcon: Fix user font detection test at fbcon_resize().
  * perf test: Free formats for perf pmu parse test
  * MIPS: SNI: Fix MIPS_L1_CACHE_SHIFT
  * perf test: Fix the "signal" test inline assembly
  * Drivers: hv: vmbus: Add timeout to vmbus_wait_for_unload
  * ASoC: qcom: Set card->owner to avoid warnings
  * clk: rockchip: Fix initialization of mux_pll_src_4plls_p
  * clk: davinci: Use the correct size when allocating memory
  * KVM: MIPS: Change the definition of kvm type
  * spi: Fix memory leak on splited transfers
  * i2c: algo: pca: Reapply i2c bus settings after reset
  * f2fs: Return EOF on unaligned end of file DIO read
  * f2fs: fix indefinite loop scanning for free nid
  * nvme-rdma: cancel async events before freeing event struct
  * nvme-fc: cancel async events before freeing event struct
  * openrisc: Fix cache API compile issue when not inlining
  * rapidio: Replace 'select' DMAENGINES 'with depends on'
  * SUNRPC: stop printk reading past end of string
  * NFS: Zero-stateid SETATTR should first return delegation
  * spi: spi-loopback-test: Fix out-of-bounds read
  * regulator: pwm: Fix machine constraints application
  * scsi: lpfc: Fix FLOGI/PLOGI receive race condition in pt2pt discovery
  * scsi: libfc: Fix for double free()
  * scsi: pm8001: Fix memleak in pm8001_exec_internal_task_abort
  * NFSv4.1 handle ERR_DELAY error reclaiming locking state on delegation recall
  * hv_netvsc: Remove "unlikely" from netvsc_select_queue
  * net: handle the return value of pskb_carve_frag_list() correctly
  * RDMA/bnxt_re: Restrict the max_gids to 256
  * gfs2: initialize transaction tr_ailX_lists earlier
  * scsi: qla2xxx: Reduce holding sess_lock to prevent CPU lock-up
  * scsi: qla2xxx: Move rport registration out of internal work_list
  * scsi: qla2xxx: Update rscn_rcvd field to more meaningful scan_needed
  * dsa: Allow forwarding of redirected IGMP traffic
  * power: smblite-lib: Reduce the ICL immediately when flash is active
  * qseecom: Propagate correct return value from TZ
  * mmc: Remove unused code
  * Merge "cnss2: change to enable self recovery"
  * Merge "cnss2: Change to add prints on link down callback"
  * ANDROID: Refresh ABI.xmls with libabigail 1.8.0-1dca710a
  * Merge "power: smb1390: Fix taper condition for VPH configuration"
  * Merge "dm-crypt: Skip encryption if bio is fscrypto or blk-crypto encrypted"
  * Merge "mmc: core: Fix issue of no clk scaling upon previous scaling failure"
  * Merge "power: smblite-lib/smb5-lib: Add partner registration for microusb otg"
  * irqchip: gic-v3: Add support to get pending irqs
  * serial: msm_geni_serial: Don't use WARN_ON for console uart
  * mmc: core: Fix issue of no clk scaling upon previous scaling failure
  * power: smb1390: Fix taper condition for VPH configuration
  * Merge "clk: qcom: rpmh: Update new clocks support on LitoMagnus"
  * Merge "usb: phy: qusb: Set the voltage to regulator according to soc capacity"
  * cnss2: Change to add prints on link down callback
  * dm-crypt: Skip encryption if bio is fscrypto or blk-crypto encrypted
  * power: smblite-lib/smb5-lib: Add partner registration for microusb otg
  * clk: qcom: rpmh: Update new clocks support on LitoMagnus
  * Merge "msm: media: uapi: Redefine NV12 format with different alignment"
  * arm: dma-mapping.c: add cpu sync in map_sg and unmap_sg
  * Merge "clk: qcom: Add BIMC logging support during kernel panic"
  * Merge "msm: ipa4: fix the unclock gsi IPA register access"
  * Merge "dt-bindings: clock: qcom: Add support for RPMH clocks"
  * defconfig: Enable blk-crypto-fallback to handle sw crypto request
  * dt-bindings: clock: qcom: Add support for RPMH clocks
  * msm: ipa4: fix the unclock gsi IPA register access
  * clk: qcom: Add BIMC logging support during kernel panic
  * dt-bindings: clock: Add support for BIMC clock
  * Merge "fuse: fix page dereference after free"
  * Merge "Merge android-4.19-stable.125 (a483478) into msm-4.19"
  * usb: core: Don't wait for completion of urbs
  * Merge "msm: kgsl: Don't skip gmufw preallocations during firmware read"
  * msm: kgsl: Enable process reclaim for A610
  * fuse: fix page dereference after free
  * msm: media: uapi: Redefine NV12 format with different alignment
  * sched/walt: Fix a potential accounting issue during window size change
  * Merge android-4.19-stable.125 (a483478) into msm-4.19
  * UPSTREAM: arm64: vdso: Build vDSO with -ffixed-x18
  * Merge da2a02489371afa74c6c1bbfa1d5e505976dad6b on remote branch
  * Merge "mhi: core: Add checks for bhie offsets"
  * mhi: core: Add checks for bhie offsets
  * Merge "icnss2: Export API to host driver to exit power save"
  * ANDROID: KMI symbol lists: migrate section name
  * usb: phy: qusb: Set the voltage to regulator according to soc capacity
  * cnss2: Collect shadow registers for RDDM scenario
  * mm: slub: Add debugfs interface to capture slub allocs owner
  * Merge "mmc: Port changes for supporting SDIO functionality to 4.19 kernel"
  * Merge 4.19.146 into android-4.19-stable
  * Linux 4.19.146
  * gcov: add support for GCC 10.1
  * usb: typec: ucsi: acpi: Check the _DEP dependencies
  * usb: Fix out of sync data toggle if a configured device is reconfigured
  * USB: serial: option: add support for SIM7070/SIM7080/SIM7090 modules
  * USB: serial: option: support dynamic Quectel USB compositions
  * USB: serial: ftdi_sio: add IDs for Xsens Mti USB converter
  * usb: core: fix slab-out-of-bounds Read in read_descriptors
  * phy: qcom-qmp: Use correct values for ipq8074 PCIe Gen2 PHY init
  * staging: greybus: audio: fix uninitialized value issue
  * video: fbdev: fix OOB read in vga_8planes_imageblit()
  * ARM: dts: vfxxx: Add syscon compatible with OCOTP
  * KVM: VMX: Don't freeze guest when event delivery causes an APIC-access exit
  * fbcon: remove now unusued 'softback_lines' cursor() argument
  * fbcon: remove soft scrollback code
  * vgacon: remove software scrollback support
  * RDMA/rxe: Fix the parent sysfs read when the interface has 15 chars
  * rbd: require global CAP_SYS_ADMIN for mapping and unmapping
  * drm/msm: Disable preemption on all 5xx targets
  * drm/tve200: Stabilize enable/disable
  * scsi: target: iscsi: Fix hang in iscsit_access_np() when getting tpg->np_login_sem
  * scsi: target: iscsi: Fix data digest calculation
  * regulator: push allocation in set_consumer_device_supply() out of lock
  * btrfs: fix wrong address when faulting in pages in the search ioctl
  * btrfs: fix lockdep splat in add_missing_dev
  * btrfs: require only sector size alignment for parent eb bytenr
  * staging: wlan-ng: fix out of bounds read in prism2sta_probe_usb()
  * iio:accel:mma8452: Fix timestamp alignment and prevent data leak.
  * iio:accel:mma7455: Fix timestamp alignment and prevent data leak.
  * iio: accel: kxsd9: Fix alignment of local buffer.
  * iio:chemical:ccs811: Fix timestamp alignment and prevent data leak.
  * iio:light:max44000 Fix timestamp alignment and prevent data leak.
  * iio:magnetometer:ak8975 Fix alignment and data leak issues.
  * iio:adc:ti-adc081c Fix alignment and data leak issues
  * iio:adc:max1118 Fix alignment of timestamp and data leak issues
  * iio:adc:ina2xx Fix timestamp alignment issue.
  * iio:adc:ti-adc084s021 Fix alignment and data leak issues.
  * iio:accel:bmc150-accel: Fix timestamp alignment and prevent data leak.
  * iio:light:ltr501 Fix timestamp alignment issue.
  * iio: adc: ti-ads1015: fix conversion when CONFIG_PM is not set
  * iio: adc: mcp3422: fix locking on error path
  * iio: adc: mcp3422: fix locking scope
  * gcov: Disable gcov build with GCC 10
  * iommu/amd: Do not use IOMMUv2 functionality when SME is active
  * drm/amdgpu: Fix bug in reporting voltage for CIK
  * ALSA: hda: fix a runtime pm issue in SOF when integrated GPU is disabled
  * cpufreq: intel_pstate: Fix intel_pstate_get_hwp_max() for turbo disabled
  * cpufreq: intel_pstate: Refuse to turn off with HWP enabled
  * ARC: [plat-hsdk]: Switch ethernet phy-mode to rgmii-id
  * HID: elan: Fix memleak in elan_input_configured
  * drivers/net/wan/hdlc_cisco: Add hard_header_len
  * HID: quirks: Set INCREMENT_USAGE_ON_DUPLICATE for all Saitek X52 devices
  * nvme-rdma: serialize controller teardown sequences
  * nvme-fabrics: don't check state NVME_CTRL_NEW for request acceptance
  * irqchip/eznps: Fix build error for !ARC700 builds
  * xfs: initialize the shortform attr header padding entry
  * drivers/net/wan/lapbether: Set network_header before transmitting
  * ALSA: hda: Fix 2 channel swapping for Tegra
  * firestream: Fix memleak in fs_open
  * NFC: st95hf: Fix memleak in st95hf_in_send_cmd
  * drivers/net/wan/lapbether: Added needed_tailroom
  * netfilter: conntrack: allow sctp hearbeat after connection re-use
  * dmaengine: acpi: Put the CSRT table after using it
  * ARC: HSDK: wireup perf irq
  * arm64: dts: ns2: Fixed QSPI compatible string
  * ARM: dts: BCM5301X: Fixed QSPI compatible string
  * ARM: dts: NSP: Fixed QSPI compatible string
  * ARM: dts: bcm: HR2: Fixed QSPI compatible string
  * mmc: sdhci-msm: Add retries when all tuning phases are found valid
  * RDMA/core: Fix reported speed and width
  * scsi: libsas: Set data_dir as DMA_NONE if libata marks qc as NODATA
  * drm/sun4i: Fix dsi dcs long write function
  * RDMA/bnxt_re: Do not report transparent vlan from QP1
  * RDMA/rxe: Drop pointless checks in rxe_init_ports
  * RDMA/rxe: Fix memleak in rxe_mem_init_user
  * ARM: dts: ls1021a: fix QuadSPI-memory reg range
  * ARM: dts: socfpga: fix register entry for timer3 on Arria10
  * ARM: dts: logicpd-som-lv-baseboard: Fix broken audio
  * ARM: dts: logicpd-torpedo-baseboard: Fix broken audio
  * msm: kgsl: Don't skip gmufw preallocations during firmware read
  * Merge "mm, oom_adj: don't loop through tasks in __set_oom_adj when not necessary"
  * icnss2: Export API to host driver to exit power save
  * Merge "msm: ipa: stay in NAPI mode when default pipe has low credits"
  * Merge "usb: typec: Fix setting of invalid value of opmode"
  * Merge "msm: kgsl: Skip SP_INST_TAG and SP_INST_DATA dump in Snapshot for A615"
  * Merge "msm: ipa3: Assert the device when IPA FW not loaded"
  * Merge "fbdev: msm: Disable EARLY_MAP setting at bootup"
  * Merge "icnss: Update QMI header file for PCIE gen switch and bdf data"
  * Merge "msm: ipa: Add graceful handling to skip partial packets"
  * Merge "clk: qcom: Update DSI clk names for SDM660"
  * Merge "fbdev: msm: Avoid adding videomode info twice"
  * Merge "msm: ipa3: Schedule the NAPI only from interrupt context"
  * Merge "uio: msm_sharedmem: improve the debug/error logs"
  * Merge "msm: fbdev: Fix compilation error for user build"
  * Merge "msm: ipa3: Fix to increase the inactivity timer"
  * Merge "msm: mink: Fix copy_to_user issue"
  * msm: ipa: stay in NAPI mode when default pipe has low credits
  * vidc_3x: Fix crash in user build
  * msm: mink: Fix copy_to_user issue
  * msm: ipa: update the iommu mapping for WDI rings
  * f2fs: clean up kvfree
  * mmc: Port changes for supporting SDIO functionality to 4.19 kernel
  * ANDROID: ABI: refresh with latest libabigail 94f5d4ae
  * uio: msm_sharedmem: improve the debug/error logs
  * cnss2: change to enable self recovery
  * msm: ipa3: Assert the device when IPA FW not loaded
  * mm, oom_adj: don't loop through tasks in __set_oom_adj when not necessary
  * Merge 4.19.145 into android-4.19-stable
  * Linux 4.19.145
  * net/mlx5e: Don't support phys switch id if not in switchdev mode
  * net: disable netpoll on fresh napis
  * tipc: fix shutdown() of connectionless socket
  * sctp: not disable bh in the whole sctp_get_port_local()
  * net: usb: dm9601: Add USB ID of Keenetic Plus DSL
  * netlabel: fix problems with mapping removal
  * block: ensure bdi->io_pages is always initialized
  * ALSA; firewire-tascam: exclude Tascam FE-8 from detection
  * fbdev: msm: Disable EARLY_MAP setting at bootup
  * f2fs: change virtual mapping way for compression pages
  * f2fs: change return value of f2fs_disable_compressed_file to bool
  * f2fs: change i_compr_blocks of inode to atomic value
  * f2fs: ignore compress mount option on image w/o compression feature
  * f2fs: allocate proper size memory for zstd decompress
  * f2fs: change compr_blocks of superblock info to 64bit
  * f2fs: add block address limit check to compressed file
  * f2fs: check position in move range ioctl
  * f2fs: correct statistic of APP_DIRECT_IO/APP_DIRECT_READ_IO
  * f2fs: support age threshold based garbage collection
  * f2fs: Use generic casefolding support
  * fs: Add standard casefolding support
  * unicode: Add utf8_casefold_hash
  * f2fs: compress: use more readable atomic_t type for {cic,dic}.ref
  * f2fs: fix compile warning
  * f2fs: support 64-bits key in f2fs rb-tree node entry
  * f2fs: inherit mtime of original block during GC
  * f2fs: record average update time of segment
  * f2fs: introduce inmem curseg
  * f2fs: compress: remove unneeded code
  * f2fs: remove duplicated type casting
  * f2fs: support zone capacity less than zone size
  * f2fs: update changes in upstream on GC_URGENT_HIGH
  * icnss: Update QMI header file for PCIE gen switch and bdf data
  * Merge "msm: kgsl: Save and restore the power ctrl_flags during recovery"
  * Merge "qdss_bridge: fix NULL usb_ch of usb_qdss_close"
  * qdss_bridge: fix NULL usb_ch of usb_qdss_close
  * Merge "cnss2: Call mhi_device_get_sync() if timeout requested is 0"
  * cnss2: Call mhi_device_get_sync() if timeout requested is 0
  * Merge "adsprpc: Remove redundant check for static pd handle"
  * Merge "usb: pd: Clear send_get_status flag in protocol reset"
  * Merge "coredump: fix crash when umh is disabled"
  * FROMGIT: binder: print warnings when detecting oneway spamming.
  * msm: kgsl: Skip SP_INST_TAG and SP_INST_DATA dump in Snapshot for A615
  * f2fs: Return EOF on unaligned end of file DIO read
  * f2fs: fix indefinite loop scanning for free nid
  * f2fs: Fix type of section block count variables
  * Merge "rpmsg: glink: Add error message in case of callback failure"
  * usb: pd: Clear send_get_status flag in protocol reset
  * Revert "arm64: kpti: force off kpti"
  * msm: cvp: fix for arbitrary command packet sending to CVP FW
  * Merge "cnss2: Add cnss_smmu_unmap API"
  * Merge "mhi: core: Free firmware memory at the end of mhi_fw_load_handler"
  * Merge 4.19.144 into android-4.19-stable
  * Linux 4.19.144
  * net: usb: Fix uninit-was-stored issue in asix_read_phy_addr()
  * cfg80211: regulatory: reject invalid hints
  * mm/hugetlb: fix a race between hugetlb sysctl handlers
  * checkpatch: fix the usage of capture group ( ... )
  * vfio/pci: Fix SR-IOV VF handling with MMIO blocking
  * KVM: arm64: Set HCR_EL2.PTW to prevent AT taking synchronous exception
  * KVM: arm64: Survive synchronous exceptions caused by AT instructions
  * KVM: arm64: Defer guest entry when an asynchronous exception is pending
  * KVM: arm64: Add kvm_extable for vaxorcism code
  * mm: slub: fix conversion of freelist_corrupted()
  * dm thin metadata: Avoid returning cmd->bm wild pointer on error
  * dm cache metadata: Avoid returning cmd->bm wild pointer on error
  * dm writecache: handle DAX to partitions on persistent memory correctly
  * libata: implement ATA_HORKAGE_MAX_TRIM_128M and apply to Sandisks
  * block: allow for_each_bvec to support zero len bvec
  * affs: fix basic permission bits to actually work
  * media: rc: uevent sysfs file races with rc_unregister_device()
  * media: rc: do not access device via sysfs after rc_unregister_device()
  * ALSA: hda - Fix silent audio output and corrupted input on MSI X570-A PRO
  * ALSA: firewire-digi00x: exclude Avid Adrenaline from detection
  * ALSA: hda/hdmi: always check pin power status in i915 pin fixup
  * ALSA: pcm: oss: Remove superfluous WARN_ON() for mulaw sanity check
  * ALSA: ca0106: fix error code handling
  * usb: qmi_wwan: add D-Link DWM-222 A2 device ID
  * net: usb: qmi_wwan: add Telit 0x1050 composition
  * btrfs: fix potential deadlock in the search ioctl
  * uaccess: Add non-pagefault user-space write function
  * uaccess: Add non-pagefault user-space read functions
  * btrfs: set the lockdep class for log tree extent buffers
  * btrfs: Remove extraneous extent_buffer_get from tree_mod_log_rewind
  * btrfs: Remove redundant extent_buffer_get in get_old_root
  * vfio-pci: Invalidate mmaps and block MMIO access on disabled memory
  * vfio-pci: Fault mmaps to enable vma tracking
  * vfio/type1: Support faulting PFNMAP vmas
  * btrfs: drop path before adding new uuid tree entry
  * xfs: don't update mtime on COW faults
  * ext2: don't update mtime on COW faults
  * include/linux/log2.h: add missing () around n in roundup_pow_of_two()
  * thermal: ti-soc-thermal: Fix bogus thermal shutdowns for omap4430
  * iommu/vt-d: Serialize IOMMU GCMD register modifications
  * x86, fakenuma: Fix invalid starting node ID
  * tg3: Fix soft lockup when tg3_reset_task() fails.
  * perf jevents: Fix suspicious code in fixregex()
  * xfs: fix xfs_bmap_validate_extent_raw when checking attr fork of rt files
  * net: gemini: Fix another missing clk_disable_unprepare() in probe
  * fix regression in "epoll: Keep a reference on files added to the check list"
  * net: ethernet: mlx4: Fix memory allocation in mlx4_buddy_init()
  * perf tools: Correct SNOOPX field offset
  * nvmet-fc: Fix a missed _irqsave version of spin_lock in 'nvmet_fc_fod_op_done()'
  * netfilter: nfnetlink: nfnetlink_unicast() reports EAGAIN instead of ENOBUFS
  * selftests/bpf: Fix massive output from test_maps
  * bnxt: don't enable NAPI until rings are ready
  * xfs: fix boundary test in xfs_attr_shortform_verify
  * bnxt_en: fix HWRM error when querying VF temperature
  * bnxt_en: Fix PCI AER error recovery flow
  * bnxt_en: Check for zero dir entries in NVRAM.
  * bnxt_en: Don't query FW when netif_running() is false.
  * gtp: add GTPA_LINK info to msg sent to userspace
  * dmaengine: pl330: Fix burst length if burst size is smaller than bus width
  * net: arc_emac: Fix memleak in arc_mdio_probe
  * ravb: Fixed to be able to unload modules
  * net: systemport: Fix memleak in bcm_sysport_probe
  * net: hns: Fix memleak in hns_nic_dev_probe
  * netfilter: nf_tables: fix destination register zeroing
  * netfilter: nf_tables: incorrect enum nft_list_attributes definition
  * netfilter: nf_tables: add NFTA_SET_USERDATA if not null
  * MIPS: BMIPS: Also call bmips_cpu_setup() for secondary cores
  * MIPS: mm: BMIPS5000 has inclusive physical caches
  * dmaengine: at_hdmac: check return value of of_find_device_by_node() in at_dma_xlate()
  * batman-adv: bla: use netif_rx_ni when not in interrupt context
  * batman-adv: Fix own OGM check in aggregated OGMs
  * batman-adv: Avoid uninitialized chaddr when handling DHCP
  * dmaengine: of-dma: Fix of_dma_router_xlate's of_dma_xlate handling
  * xen/xenbus: Fix granting of vmalloc'd memory
  * s390: don't trace preemption in percpu macros
  * cpuidle: Fixup IRQ state
  * ceph: don't allow setlease on cephfs
  * drm/msm/a6xx: fix gmu start on newer firmware
  * nvmet: Disable keep-alive timer when kato is cleared to 0h
  * hwmon: (applesmc) check status earlier.
  * drm/msm: add shutdown support for display platform_driver
  * tty: serial: qcom_geni_serial: Drop __init from qcom_geni_console_setup
  * scsi: target: tcmu: Optimize use of flush_dcache_page
  * scsi: target: tcmu: Fix size in calls to tcmu_flush_dcache_range
  * perf record/stat: Explicitly call out event modifiers in the documentation
  * HID: core: Sanitize event code and type when mapping input
  * HID: core: Correctly handle ReportSize being zero
  * msm: ipa: Add graceful handling to skip partial packets
  * Merge "scsi: ufs: fix ufs power down specs violation in shutdown path"
  * usb: typec: Fix setting of invalid value of opmode
  * Merge "mhi: core: Add missing EXPORT_SYMBOL for some public APIs"
  * cnss2: Add cnss_smmu_unmap API
  * mhi: core: Add missing EXPORT_SYMBOL for some public APIs
  * cnss2: Notify MSM PCIe bus driver when detect link down
  * pci: msm: Add support to handle ep driver requested link down
  * scsi: ufs: fix ufs power down specs violation in shutdown path
  * Merge "iommu: arm-smmu: update copyright year to retain older copyright year"
  * rpmsg: glink: Add error message in case of callback failure
  * msm: ipa3: Fix to increase the inactivity timer
  * coredump: fix crash when umh is disabled
  * adsprpc: Remove redundant check for static pd handle
  * Merge "soc: qcom: Fix mhi_qdss mode permission issue"
  * Merge "msm: cvp: Enhance CVP release persist buffer process for DSP sessions"
  * Merge "sched/fair: Improve the scheduler"
  * Merge "msm: ipa: disable status on ODL_DPL_CONS pipe above IPAv4.5 targets"
  * msm: cvp: Enhance CVP release persist buffer process for DSP sessions
  * soc: qcom: Fix mhi_qdss mode permission issue
  * Merge "kernel: dma: disallow altering logical map for non-CMA allocs"
  * Merge "icnss: Vote for chain1 regulators if RD card is 2x2"
  * clk: qcom: Update DSI clk names for SDM660
  * Merge "drivers: thermal: notify at least one thermal zone if no trips are violated"
  * Merge "cnss2: Add support to use new ramdump APIs"
  * Merge "defconfig: msm: Enable BCL sensor for SDM660"
  * defconfig: msm: Enable BCL sensor for SDM660
  * Merge "icnss: Get RD card chain mask capability as part of qmi cap resp"
  * icnss: Vote for chain1 regulators if RD card is 2x2
  * Merge "mm: fix the page_owner initializing issue for arm32"
  * Merge "leds: qti-flash: Enable the torch mode after enabling the led module"
  * Merge "serial: msm_geni_serial: Log UART CTS and RX IO line status"
  * Merge "defconfig: msm: disable CONFIG_CRYPTO_MD4 to align with android base"
  * serial: msm_geni_serial: Log UART CTS and RX IO line status
  * defconfig: msm: disable CONFIG_CRYPTO_MD4 to align with android base
  * Merge "sched/fair: Improve the scheduler"
  * Merge "scsi: ufs: Fix per-LU WB buffer size setting error"
  * leds: qti-flash: Enable the torch mode after enabling the led module
  * Merge "cnss2: during recovery update status only when fw is running"
  * Merge "soc: qcom: update failure reason response message"
  * sched/fair: Improve the scheduler
  * scsi: ufs: Fix per-LU WB buffer size setting error
  * cnss2: Add runtime PM stats support
  * drivers: thermal: notify at least one thermal zone if no trips are violated
  * Merge "cnss2: change for fixed link speed"
  * Merge 4.19.143 into android-4.19-stable
  * Linux 4.19.143
  * ALSA: usb-audio: Update documentation comment for MS2109 quirk
  * HID: hiddev: Fix slab-out-of-bounds write in hiddev_ioctl_usage()
  * tpm: Unify the mismatching TPM space buffer sizes
  * usb: dwc3: gadget: Handle ZLP for sg requests
  * usb: dwc3: gadget: Fix handling ZLP
  * usb: dwc3: gadget: Don't setup more than requested
  * btrfs: check the right error variable in btrfs_del_dir_entries_in_log
  * usb: storage: Add unusual_uas entry for Sony PSZ drives
  * USB: cdc-acm: rework notification_buffer resizing
  * USB: gadget: u_f: Unbreak offset calculation in VLAs
  * USB: gadget: f_ncm: add bounds checks to ncm_unwrap_ntb()
  * USB: gadget: u_f: add overflow checks to VLA macros
  * usb: host: ohci-exynos: Fix error handling in exynos_ohci_probe()
  * USB: Ignore UAS for JMicron JMS567 ATA/ATAPI Bridge
  * USB: quirks: Ignore duplicate endpoint on Sound Devices MixPre-D
  * USB: quirks: Add no-lpm quirk for another Raydium touchscreen
  * usb: uas: Add quirk for PNY Pro Elite
  * USB: yurex: Fix bad gfp argument
  * drm/amd/pm: correct Vega12 swctf limit setting
  * drm/amd/pm: correct Vega10 swctf limit setting
  * drm/amdgpu: Fix buffer overflow in INFO ioctl
  * irqchip/stm32-exti: Avoid losing interrupts due to clearing pending bits by mistake
  * genirq/matrix: Deal with the sillyness of for_each_cpu() on UP
  * device property: Fix the secondary firmware node handling in set_primary_fwnode()
  * PM: sleep: core: Fix the handling of pending runtime resume requests
  * xhci: Always restore EP_SOFT_CLEAR_TOGGLE even if ep reset failed
  * xhci: Do warm-reset when both CAS and XDEV_RESUME are set
  * usb: host: xhci: fix ep context print mismatch in debugfs
  * XEN uses irqdesc::irq_data_common::handler_data to store a per interrupt XEN data pointer which contains XEN specific information.
  * writeback: Fix sync livelock due to b_dirty_time processing
  * writeback: Avoid skipping inode writeback
  * writeback: Protect inode->i_io_list with inode->i_lock
  * serial: 8250: change lock order in serial8250_do_startup()
  * serial: 8250_exar: Fix number of ports for Commtech PCIe cards
  * serial: pl011: Don't leak amba_ports entry on driver register error
  * serial: pl011: Fix oops on -EPROBE_DEFER
  * serial: samsung: Removes the IRQ not found warning
  * vt_ioctl: change VT_RESIZEX ioctl to check for error return from vc_resize()
  * vt: defer kfree() of vc_screenbuf in vc_do_resize()
  * USB: lvtest: return proper error code in probe
  * fbcon: prevent user font height or width change from causing potential out-of-bounds access
  * btrfs: fix space cache memory leak after transaction abort
  * btrfs: reset compression level for lzo on remount
  * blk-mq: order adding requests to hctx->dispatch and checking SCHED_RESTART
  * HID: i2c-hid: Always sleep 60ms after I2C_HID_PWR_ON commands
  * block: loop: set discard granularity and alignment for block device backed loop
  * powerpc/perf: Fix soft lockups due to missed interrupt accounting
  * net: gianfar: Add of_node_put() before goto statement
  * macvlan: validate setting of multiple remote source MAC addresses
  * Revert "scsi: qla2xxx: Fix crash on qla2x00_mailbox_command"
  * scsi: qla2xxx: Fix null pointer access during disconnect from subsystem
  * scsi: qla2xxx: Check if FW supports MQ before enabling
  * scsi: ufs: Clean up completed request without interrupt notification
  * scsi: ufs: Improve interrupt handling for shared interrupts
  * scsi: ufs: Fix possible infinite loop in ufshcd_hold
  * scsi: fcoe: Fix I/O path allocation
  * ASoC: wm8994: Avoid attempts to read unreadable registers
  * s390/cio: add cond_resched() in the slow_eval_known_fn() loop
  * spi: stm32: fix stm32_spi_prepare_mbr in case of odd clk_rate
  * fs: prevent BUG_ON in submit_bh_wbc()
  * ext4: correctly restore system zone info when remount fails
  * ext4: handle error of ext4_setup_system_zone() on remount
  * ext4: handle option set by mount flags correctly
  * jbd2: abort journal if free a async write error metadata buffer
  * ext4: handle read only external journal device
  * ext4: don't BUG on inconsistent journal feature
  * jbd2: make sure jh have b_transaction set in refile/unfile_buffer
  * usb: gadget: f_tcm: Fix some resource leaks in some error paths
  * i2c: rcar: in slave mode, clear NACK earlier
  * null_blk: fix passing of REQ_FUA flag in null_handle_rq
  * nvme-fc: Fix wrong return value in __nvme_fc_init_request()
  * drm/msm/adreno: fix updating ring fence
  * media: gpio-ir-tx: improve precision of transmitted signal due to scheduling
  * Revert "ath10k: fix DMA related firmware crashes on multiple devices"
  * efi: provide empty efi_enter_virtual_mode implementation
  * USB: sisusbvga: Fix a potential UB casued by left shifting a negative value
  * powerpc/spufs: add CONFIG_COREDUMP dependency
  * KVM: arm64: Fix symbol dependency in __hyp_call_panic_nvhe
  * EDAC/ie31200: Fallback if host bridge device is already initialized
  * scsi: fcoe: Memory leak fix in fcoe_sysfs_fcf_del()
  * ceph: fix potential mdsc use-after-free crash
  * scsi: iscsi: Do not put host in iscsi_set_flashnode_param()
  * btrfs: file: reserve qgroup space after the hole punch range is locked
  * locking/lockdep: Fix overflow in presentation of average lock-time
  * drm/nouveau: Fix reference count leak in nouveau_connector_detect
  * drm/nouveau: fix reference count leak in nv50_disp_atomic_commit
  * drm/nouveau/drm/noveau: fix reference count leak in nouveau_fbcon_open
  * f2fs: fix use-after-free issue
  * HID: quirks: add NOGET quirk for Logitech GROUP
  * cec-api: prevent leaking memory through hole in structure
  * mips/vdso: Fix resource leaks in genvdso.c
  * rtlwifi: rtl8192cu: Prevent leaking urb
  * ARM: dts: ls1021a: output PPS signal on FIPER2
  * PCI: Fix pci_create_slot() reference count leak
  * omapfb: fix multiple reference count leaks due to pm_runtime_get_sync
  * f2fs: fix error path in do_recover_data()
  * selftests/powerpc: Purge extra count_pmc() calls of ebb selftests
  * xfs: Don't allow logging of XFS_ISTALE inodes
  * scsi: lpfc: Fix shost refcount mismatch when deleting vport
  * drm/amdgpu/display: fix ref count leak when pm_runtime_get_sync fails
  * drm/amdgpu: fix ref count leak in amdgpu_display_crtc_set_config
  * drm/amd/display: fix ref count leak in amdgpu_drm_ioctl
  * drm/amdgpu: fix ref count leak in amdgpu_driver_open_kms
  * drm/radeon: fix multiple reference count leak
  * drm/amdkfd: Fix reference count leaks.
  * iommu/iova: Don't BUG on invalid PFNs
  * scsi: target: tcmu: Fix crash on ARM during cmd completion
  * blktrace: ensure our debugfs dir exists
  * media: pci: ttpci: av7110: fix possible buffer overflow caused by bad DMA value in debiirq()
  * powerpc/xive: Ignore kmemleak false positives
  * arm64: dts: qcom: msm8916: Pull down PDM GPIOs during sleep
  * mfd: intel-lpss: Add Intel Emmitsburg PCH PCI IDs
  * ASoC: tegra: Fix reference count leaks.
  * ASoC: img-parallel-out: Fix a reference count leak
  * ASoC: img: Fix a reference count leak in img_i2s_in_set_fmt
  * ALSA: pci: delete repeated words in comments
  * ipvlan: fix device features
  * net: ena: Make missed_tx stat incremental
  * tipc: fix uninit skb->data in tipc_nl_compat_dumpit()
  * net/smc: Prevent kernel-infoleak in __smc_diag_dump()
  * net: qrtr: fix usage of idr in port assignment to socket
  * net: Fix potential wrong skb->protocol in skb_vlan_untag()
  * gre6: Fix reception with IP6_TNL_F_RCV_DSCP_COPY
  * powerpc/64s: Don't init FSCR_DSCR in __init_FSCR()
  * Merge "dcc_v2: Add dcc region to minidump table"
  * Merge "usb: dwc3: gadget: Fix request completion check"
  * kernel: dma: disallow altering logical map for non-CMA allocs
  * Merge "leds: qti-flash: Disable flash LED channel after De-strobe"
  * Merge "msm: kgsl: Add support to vote bimc-interface-clk per speed-bin"
  * Merge "msm: ipa: set the mac header to 0 for rx skbs"
  * Merge "clk: qcom: mdss: DP PLL changes for SDM660"
  * cnss2: during recovery update status only when fw is running
  * ANDROID: gki_defconfig: initialize locals with zeroes
  * UPSTREAM: security: allow using Clang's zero initialization for stack variables
  * msm: ipa: disable status on ODL_DPL_CONS pipe above IPAv4.5 targets
  * icnss: Get RD card chain mask capability as part of qmi cap resp
  * Merge "usb: phy: qusb2: Update tune params based on efuse value"
  * msm: kgsl: Save and restore the power ctrl_flags during recovery
  * mm: fix the page_owner initializing issue for arm32
  * Merge "usb: pd: Reset pd protocol during cable disconnect"
  * sched/fair: Improve the scheduler
  * Merge "cnss2: Remove hard-code address for error log start for HST"
  * Merge "cnss2: Set PCIe link state in proper state during resume"
  * Merge "mhi: net: Reduce NAPI poll weight to 64 for debug builds"
  * mhi: net: Reduce NAPI poll weight to 64 for debug builds
  * usb: pd: Reset pd protocol during cable disconnect
  * cnss2: change for fixed link speed
  * usb: phy: qusb2: Update tune params based on efuse value
  * Merge "defconfig: bengal: Enable Incremental FS support"
  * Merge "backlight: qcom-spmi-wled: Force HFRC off when WLED is disabled"
  * dcc_v2: Add dcc region to minidump table
  * Merge "defconfig: enable fixed-regulator driver for 32-bit Bengal"
  * Merge "diag: Continue through the loop if condition fails for a proc"
  * defconfig: bengal: Enable Incremental FS support
  * defconfig: enable fixed-regulator driver for 32-bit Bengal
  * ANDROID: Incremental fs: magic number compatible 32-bit
  * diag: Continue through the loop if condition fails for a proc
  * msm: kgsl: Add support to vote bimc-interface-clk per speed-bin
  * msm: fbdev: Fix compilation error for user build
  * clk: qcom: mdss: DP PLL changes for SDM660
  * cnss2: Remove hard-code address for error log start for HST
  * soc: qcom: update failure reason response message
  * Revert "binder: Prevent context manager from incrementing ref 0"
  * ANDROID: GKI: update the ABI xml
  * usb: dwc3: gadget: Fix request completion check
  * cnss2: Set PCIe link state in proper state during resume
  * cnss2: Trigger recovery when link is down before force RDDM
  * BACKPORT: recordmcount: support >64k sections
  * UPSTREAM: arm64: vdso: Build vDSO with -ffixed-x18
  * fbdev: msm: Avoid adding videomode info twice
  * UPSTREAM: cgroup: Remove unused cgrp variable
  * UPSTREAM: cgroup: freezer: call cgroup_enter_frozen() with preemption disabled in ptrace_stop()
  * UPSTREAM: cgroup: freezer: fix frozen state inheritance
  * UPSTREAM: signal: unconditionally leave the frozen state in ptrace_stop()
  * BACKPORT: cgroup: cgroup v2 freezer
  * UPSTREAM: cgroup: implement __cgroup_task_count() helper
  * UPSTREAM: cgroup: rename freezer.c into legacy_freezer.c
  * UPSTREAM: cgroup: remove extra cgroup_migrate_finish() call
  * UPSTREAM: cgroup: saner refcounting for cgroup_root
  * UPSTREAM: cgroup: Add named hierarchy disabling to cgroup_no_v1 boot param
  * UPSTREAM: cgroup: remove unnecessary unlikely()
  * UPSTREAM: cgroup: Simplify cgroup_ancestor
  * Merge 4.19.142 into android-4.19-stable
  * Linux 4.19.142
  * KVM: arm64: Only reschedule if MMU_NOTIFIER_RANGE_BLOCKABLE is not set
  * KVM: Pass MMU notifier range flags to kvm_unmap_hva_range()
  * clk: Evict unregistered clks from parent caches
  * xen: don't reschedule in preemption off sections
  * mm/hugetlb: fix calculation of adjust_range_if_pmd_sharing_possible
  * do_epoll_ctl(): clean the failure exits up a bit
  * epoll: Keep a reference on files added to the check list
  * efi: add missed destroy_workqueue when efisubsys_init fails
  * powerpc/pseries: Do not initiate shutdown when system is running on UPS
  * net: dsa: b53: check for timeout
  * hv_netvsc: Fix the queue_mapping in netvsc_vf_xmit()
  * net: gemini: Fix missing free_netdev() in error path of gemini_ethernet_port_probe()
  * net: ena: Prevent reset after device destruction
  * bonding: fix active-backup failover for current ARP slave
  * afs: Fix NULL deref in afs_dynroot_depopulate()
  * RDMA/bnxt_re: Do not add user qps to flushlist
  * Fix build error when CONFIG_ACPI is not set/enabled:
  * efi: avoid error message when booting under Xen
  * kconfig: qconf: fix signal connection to invalid slots
  * kconfig: qconf: do not limit the pop-up menu to the first row
  * kvm: x86: Toggling CR4.PKE does not load PDPTEs in PAE mode
  * kvm: x86: Toggling CR4.SMAP does not load PDPTEs in PAE mode
  * vfio/type1: Add proper error unwind for vfio_iommu_replay()
  * ASoC: intel: Fix memleak in sst_media_open
  * ASoC: msm8916-wcd-analog: fix register Interrupt offset
  * s390/ptrace: fix storage key handling
  * s390/runtime_instrumentation: fix storage key handling
  * bonding: fix a potential double-unregister
  * bonding: show saner speed for broadcast mode
  * net: fec: correct the error path for regulator disable in probe
  * i40e: Fix crash during removing i40e driver
  * i40e: Set RX_ONLY mode for unicast promiscuous on VLAN
  * ASoC: q6routing: add dummy register read/write function
  * ext4: don't allow overlapping system zones
  * ext4: fix potential negative array index in do_split()
  * fs/signalfd.c: fix inconsistent return codes for signalfd4
  * alpha: fix annotation of io{read,write}{16,32}be()
  * xfs: Fix UBSAN null-ptr-deref in xfs_sysfs_init
  * tools/testing/selftests/cgroup/cgroup_util.c: cg_read_strcmp: fix null pointer dereference
  * virtio_ring: Avoid loop when vq is broken in virtqueue_poll
  * scsi: libfc: Free skb in fc_disc_gpn_id_resp() for valid cases
  * cpufreq: intel_pstate: Fix cpuinfo_max_freq when MSR_TURBO_RATIO_LIMIT is 0
  * ceph: fix use-after-free for fsc->mdsc
  * jffs2: fix UAF problem
  * xfs: fix inode quota reservation checks
  * svcrdma: Fix another Receive buffer leak
  * m68knommu: fix overwriting of bits in ColdFire V3 cache control
  * Input: psmouse - add a newline when printing 'proto' by sysfs
  * media: vpss: clean up resources in init
  * rtc: goldfish: Enable interrupt in set_alarm() when necessary
  * media: budget-core: Improve exception handling in budget_register()
  * scsi: target: tcmu: Fix crash in tcmu_flush_dcache_range on ARM
  * scsi: ufs: Add DELAY_BEFORE_LPM quirk for Micron devices
  * spi: Prevent adding devices below an unregistering controller
  * kthread: Do not preempt current task if it is going to call schedule()
  * drm/amd/display: fix pow() crashing when given base 0
  * scsi: zfcp: Fix use-after-free in request timeout handlers
  * jbd2: add the missing unlock_buffer() in the error path of jbd2_write_superblock()
  * ext4: fix checking of directory entry validity for inline directories
  * mm, page_alloc: fix core hung in free_pcppages_bulk()
  * mm: include CMA pages in lowmem_reserve at boot
  * kernel/relay.c: fix memleak on destroy relay channel
  * romfs: fix uninitialized memory leak in romfs_dev_read()
  * btrfs: sysfs: use NOFS for device creation
  * btrfs: inode: fix NULL pointer dereference if inode doesn't need compression
  * btrfs: Move free_pages_out label in inline extent handling branch in compress_file_range
  * btrfs: don't show full path of bind mounts in subvol=
  * btrfs: export helpers for subvolume name/id resolution
  * khugepaged: adjust VM_BUG_ON_MM() in __khugepaged_enter()
  * khugepaged: khugepaged_test_exit() check mmget_still_valid()
  * perf probe: Fix memory leakage when the probe point is not found
  * drm/vgem: Replace opencoded version of drm_gem_dumb_map_offset()
  * leds: qti-flash: Disable flash LED channel after De-strobe
  * leds: qti-flash: Disable flash LED when brightness is set to 0
  * iommu: arm-smmu: update copyright year to retain older copyright year
  * ANDROID: tty: fix tty name overflow
  * ANDROID: Revert "PCI: Probe bridge window attributes once at enumeration-time"
  * Merge 4.19.141 into android-4.19-stable
  * Restore sdcardfs feature
  * Linux 4.19.141
  * drm/amdgpu: Fix bug where DPM is not enabled after hibernate and resume
  * drm: Added orientation quirk for ASUS tablet model T103HAF
  * arm64: dts: marvell: espressobin: add ethernet alias
  * khugepaged: retract_page_tables() remember to test exit
  * sh: landisk: Add missing initialization of sh_io_port_base
  * tools build feature: Quote CC and CXX for their arguments
  * perf bench mem: Always memset source before memcpy
  * ALSA: echoaudio: Fix potential Oops in snd_echo_resume()
  * mfd: dln2: Run event handler loop under spinlock
  * test_kmod: avoid potential double free in trigger_config_run_type()
  * fs/ufs: avoid potential u32 multiplication overflow
  * fs/minix: remove expected error message in block_to_path()
  * fs/minix: fix block limit check for V1 filesystems
  * fs/minix: set s_maxbytes correctly
  * nfs: Fix getxattr kernel panic and memory overflow
  * net: qcom/emac: add missed clk_disable_unprepare in error path of emac_clks_phase1_init
  * drm/vmwgfx: Fix two list_for_each loop exit tests
  * drm/vmwgfx: Use correct vmw_legacy_display_unit pointer
  * Input: sentelic - fix error return when fsp_reg_write fails
  * watchdog: initialize device before misc_register
  * scsi: lpfc: nvmet: Avoid hang / use-after-free again when destroying targetport
  * openrisc: Fix oops caused when dumping stack
  * i2c: rcar: avoid race when unregistering slave
  * tools build feature: Use CC and CXX from parent
  * pwm: bcm-iproc: handle clk_get_rate() return
  * clk: clk-atlas6: fix return value check in atlas6_clk_init()
  * i2c: rcar: slave: only send STOP event when we have been addressed
  * iommu/vt-d: Enforce PASID devTLB field mask
  * iommu/omap: Check for failure of a call to omap_iommu_dump_ctx
  * selftests/powerpc: ptrace-pkey: Don't update expected UAMOR value
  * selftests/powerpc: ptrace-pkey: Update the test to mark an invalid pkey correctly
  * selftests/powerpc: ptrace-pkey: Rename variables to make it easier to follow code
  * dm rq: don't call blk_mq_queue_stopped() in dm_stop_queue()
  * gpu: ipu-v3: image-convert: Combine rotate/no-rotate irq handlers
  * mmc: renesas_sdhi_internal_dmac: clean up the code for dma complete
  * USB: serial: ftdi_sio: clean up receive processing
  * USB: serial: ftdi_sio: make process-packet buffer unsigned
  * media: rockchip: rga: Only set output CSC mode for RGB input
  * media: rockchip: rga: Introduce color fmt macros and refactor CSC mode logic
  * RDMA/ipoib: Fix ABBA deadlock with ipoib_reap_ah()
  * RDMA/ipoib: Return void from ipoib_ib_dev_stop()
  * mfd: arizona: Ensure 32k clock is put on driver unbind and error
  * drm/imx: imx-ldb: Disable both channels for split mode in enc->disable()
  * remoteproc: qcom: q6v5: Update running state before requesting stop
  * perf intel-pt: Fix FUP packet state
  * module: Correctly truncate sysfs sections output
  * pseries: Fix 64 bit logical memory block panic
  * watchdog: f71808e_wdt: clear watchdog timeout occurred flag
  * watchdog: f71808e_wdt: remove use of wrong watchdog_info option
  * watchdog: f71808e_wdt: indicate WDIOF_CARDRESET support in watchdog_info.options
  * tracing: Use trace_sched_process_free() instead of exit() for pid tracing
  * tracing/hwlat: Honor the tracing_cpumask
  * kprobes: Fix NULL pointer dereference at kprobe_ftrace_handler
  * ftrace: Setup correct FTRACE_FL_REGS flags for module
  * mm/page_counter.c: fix protection usage propagation
  * ocfs2: change slot number type s16 to u16
  * ext2: fix missing percpu_counter_inc
  * MIPS: CPU#0 is not hotpluggable
  * driver core: Avoid binding drivers to dead devices
  * mac80211: fix misplaced while instead of if
  * bcache: fix overflow in offset_to_stripe()
  * bcache: allocate meta data pages as compound pages
  * md/raid5: Fix Force reconstruct-write io stuck in degraded raid5
  * net/compat: Add missing sock updates for SCM_RIGHTS
  * net: stmmac: dwmac1000: provide multicast filter fallback
  * net: ethernet: stmmac: Disable hardware multicast filter
  * media: vsp1: dl: Fix NULL pointer dereference on unbind
  * powerpc: Fix circular dependency between percpu.h and mmu.h
  * powerpc: Allow 4224 bytes of stack expansion for the signal frame
  * cifs: Fix leak when handling lease break for cached root fid
  * xtensa: fix xtensa_pmu_setup prototype
  * iio: dac: ad5592r: fix unbalanced mutex unlocks in ad5592r_read_raw()
  * dt-bindings: iio: io-channel-mux: Fix compatible string in example code
  * btrfs: fix return value mixup in btrfs_get_extent
  * btrfs: fix memory leaks after failure to lookup checksums during inode logging
  * btrfs: only search for left_info if there is no right_info in try_merge_free_space
  * btrfs: fix messages after changing compression level by remount
  * btrfs: open device without device_list_mutex
  * btrfs: don't traverse into the seed devices in show_devname
  * btrfs: ref-verify: fix memory leak in add_block_entry
  * btrfs: don't allocate anonymous block device for user invisible roots
  * btrfs: free anon block device right after subvolume deletion
  * PCI: Probe bridge window attributes once at enumeration-time
  * PCI: qcom: Add support for tx term offset for rev 2.1.0
  * PCI: qcom: Define some PARF params needed for ipq8064 SoC
  * PCI: Add device even if driver attach failed
  * PCI: Mark AMD Navi10 GPU rev 0x00 ATS as broken
  * PCI: hotplug: ACPI: Fix context refcounting in acpiphp_grab_context()
  * genirq/affinity: Make affinity setting if activated opt-in
  * smb3: warn on confusing error scenario with sec=krb5
  * ANDROID: ABI: update the ABI xml representation
  * Revert "ALSA: usb-audio: work around streaming quirk for MacroSilicon MS2109"
  * Merge 4.19.140 into android-4.19-stable
  * Linux 4.19.140
  * xen/gntdev: Fix dmabuf import with non-zero sgt offset
  * xen/balloon: make the balloon wait interruptible
  * xen/balloon: fix accounting in alloc_xenballooned_pages error path
  * irqdomain/treewide: Free firmware node after domain removal
  * ARM: 8992/1: Fix unwind_frame for clang-built kernels
  * parisc: mask out enable and reserved bits from sba imask
  * parisc: Implement __smp_store_release and __smp_load_acquire barriers
  * mtd: rawnand: qcom: avoid write to unavailable register
  * spi: spidev: Align buffers for DMA
  * include/asm-generic/vmlinux.lds.h: align ro_after_init
  * cpufreq: dt: fix oops on armada37xx
  * NFS: Don't return layout segments that are in use
  * NFS: Don't move layouts to plh_return_segs list while in use
  * drm/ttm/nouveau: don't call tt destroy callback on alloc failure.
  * 9p: Fix memory leak in v9fs_mount
  * ALSA: usb-audio: add quirk for Pioneer DDJ-RB
  * fs/minix: reject too-large maximum file size
  * fs/minix: don't allow getting deleted inodes
  * fs/minix: check return value of sb_getblk()
  * bitfield.h: don't compile-time validate _val in FIELD_FIT
  * crypto: cpt - don't sleep of CRYPTO_TFM_REQ_MAY_SLEEP was not specified
  * crypto: ccp - Fix use of merged scatterlists
  * crypto: qat - fix double free in qat_uclo_create_batch_init_list
  * crypto: hisilicon - don't sleep of CRYPTO_TFM_REQ_MAY_SLEEP was not specified
  * pstore: Fix linking when crypto API disabled
  * ALSA: usb-audio: work around streaming quirk for MacroSilicon MS2109
  * ALSA: usb-audio: fix overeager device match for MacroSilicon MS2109
  * ALSA: usb-audio: Creative USB X-Fi Pro SB1095 volume knob support
  * ALSA: hda - fix the micmute led status for Lenovo ThinkCentre AIO
  * USB: serial: cp210x: enable usb generic throttle/unthrottle
  * USB: serial: cp210x: re-enable auto-RTS on open
  * net: initialize fastreuse on inet_inherit_port
  * net: refactor bind_bucket fastreuse into helper
  * net/tls: Fix kmap usage
  * net: Set fput_needed iff FDPUT_FPUT is set
  * net/nfc/rawsock.c: add CAP_NET_RAW check.
  * drivers/net/wan/lapbether: Added needed_headroom and a skb->len check
  * af_packet: TPACKET_V3: fix fill status rwlock imbalance
  * crypto: aesni - add compatibility with IAS
  * x86/fsgsbase/64: Fix NULL deref in 86_fsgsbase_read_task
  * svcrdma: Fix page leak in svc_rdma_recv_read_chunk()
  * pinctrl-single: fix pcs_parse_pinconf() return value
  * ocfs2: fix unbalanced locking
  * dlm: Fix kobject memleak
  * fsl/fman: fix eth hash table allocation
  * fsl/fman: check dereferencing null pointer
  * fsl/fman: fix unreachable code
  * fsl/fman: fix dereference null return value
  * fsl/fman: use 32-bit unsigned integer
  * net: spider_net: Fix the size used in a 'dma_free_coherent()' call
  * liquidio: Fix wrong return value in cn23xx_get_pf_num()
  * net: ethernet: aquantia: Fix wrong return value
  * tools, build: Propagate build failures from tools/build/Makefile.build
  * wl1251: fix always return 0 error
  * s390/qeth: don't process empty bridge port events
  * ASoC: meson: axg-tdm-interface: fix link fmt setup
  * selftests/powerpc: Fix online CPU selection
  * PCI: Release IVRS table in AMD ACS quirk
  * selftests/powerpc: Fix CPU affinity for child process
  * powerpc/boot: Fix CONFIG_PPC_MPC52XX references
  * net: dsa: rtl8366: Fix VLAN set-up
  * net: dsa: rtl8366: Fix VLAN semantics
  * Bluetooth: hci_serdev: Only unregister device if it was registered
  * Bluetooth: hci_h5: Set HCI_UART_RESET_ON_INIT to correct flags
  * power: supply: check if calc_soc succeeded in pm860x_init_battery
  * Smack: prevent underflow in smk_set_cipso()
  * Smack: fix another vsscanf out of bounds
  * RDMA/core: Fix return error value in _ib_modify_qp() to negative
  * PCI: cadence: Fix updating Vendor ID and Subsystem Vendor ID register
  * net: dsa: mv88e6xxx: MV88E6097 does not support jumbo configuration
  * scsi: mesh: Fix panic after host or bus reset
  * usb: dwc2: Fix error path in gadget registration
  * MIPS: OCTEON: add missing put_device() call in dwc3_octeon_device_init()
  * coresight: tmc: Fix TMC mode read in tmc_read_unprepare_etb()
  * thermal: ti-soc-thermal: Fix reversed condition in ti_thermal_expose_sensor()
  * usb: core: fix quirks_param_set() writing to a const pointer
  * USB: serial: iuu_phoenix: fix led-activity helpers
  * drm/imx: tve: fix regulator_disable error path
  * powerpc/book3s64/pkeys: Use PVR check instead of cpu feature
  * PCI/ASPM: Add missing newline in sysfs 'policy'
  * staging: rtl8192u: fix a dubious looking mask before a shift
  * RDMA/rxe: Prevent access to wr->next ptr afrer wr is posted to send queue
  * RDMA/qedr: SRQ's bug fixes
  * powerpc/vdso: Fix vdso cpu truncation
  * mwifiex: Prevent memory corruption handling keys
  * scsi: scsi_debug: Add check for sdebug_max_queue during module init
  * drm/bridge: sil_sii8620: initialize return of sii8620_readb
  * phy: exynos5-usbdrd: Calibrating makes sense only for USB2.0 PHY
  * drm: panel: simple: Fix bpc for LG LB070WV8 panel
  * leds: core: Flush scheduled work for system suspend
  * PCI: Fix pci_cfg_wait queue locking problem
  * RDMA/rxe: Skip dgid check in loopback mode
  * xfs: fix reflink quota reservation accounting error
  * xfs: don't eat an EIO/ENOSPC writeback error when scrubbing data fork
  * media: exynos4-is: Add missed check for pinctrl_lookup_state()
  * media: firewire: Using uninitialized values in node_probe()
  * ipvs: allow connection reuse for unconfirmed conntrack
  * scsi: eesox: Fix different dev_id between request_irq() and free_irq()
  * scsi: powertec: Fix different dev_id between request_irq() and free_irq()
  * drm/radeon: fix array out-of-bounds read and write issues
  * cxl: Fix kobject memleak
  * drm/mipi: use dcs write for mipi_dsi_dcs_set_tear_scanline
  * scsi: cumana_2: Fix different dev_id between request_irq() and free_irq()
  * ASoC: Intel: bxt_rt298: add missing .owner field
  * media: omap3isp: Add missed v4l2_ctrl_handler_free() for preview_init_entities()
  * leds: lm355x: avoid enum conversion warning
  * drm/arm: fix unintentional integer overflow on left shift
  * drm/etnaviv: Fix error path on failure to enable bus clk
  * iio: improve IIO_CONCENTRATION channel type description
  * ath10k: Acquire tx_lock in tx error paths
  * video: pxafb: Fix the function used to balance a 'dma_alloc_coherent()' call
  * console: newport_con: fix an issue about leak related system resources
  * video: fbdev: sm712fb: fix an issue about iounmap for a wrong address
  * agp/intel: Fix a memory leak on module initialisation failure
  * drm/msm: ratelimit crtc event overflow error
  * ACPICA: Do not increment operation_region reference counts for field units
  * bcache: fix super block seq numbers comparision in register_cache_set()
  * dyndbg: fix a BUG_ON in ddebug_describe_flags
  * usb: bdc: Halt controller on suspend
  * bdc: Fix bug causing crash after multiple disconnects
  * usb: gadget: net2280: fix memory leak on probe error handling paths
  * gpu: host1x: debug: Fix multiple channels emitting messages simultaneously
  * iwlegacy: Check the return value of pcie_capability_read_*()
  * brcmfmac: set state of hanger slot to FREE when flushing PSQ
  * brcmfmac: To fix Bss Info flag definition Bug
  * brcmfmac: keep SDIO watchdog running when console_interval is non-zero
  * mm/mmap.c: Add cond_resched() for exit_mmap() CPU stalls
  * irqchip/irq-mtk-sysirq: Replace spinlock with raw_spinlock
  * drm/radeon: disable AGP by default
  * drm/debugfs: fix plain echo to connector "force" attribute
  * usb: mtu3: clear dual mode of u3port when disable device
  * drm/nouveau: fix multiple instances of reference count leaks
  * drm/etnaviv: fix ref count leak via pm_runtime_get_sync
  * arm64: dts: hisilicon: hikey: fixes to comply with adi, adv7533 DT binding
  * md-cluster: fix wild pointer of unlock_all_bitmaps()
  * video: fbdev: neofb: fix memory leak in neo_scan_monitor()
  * crypto: aesni - Fix build with LLVM_IAS=1
  * drm/radeon: Fix reference count leaks caused by pm_runtime_get_sync
  * drm/amdgpu: avoid dereferencing a NULL pointer
  * fs/btrfs: Add cond_resched() for try_release_extent_mapping() stalls
  * loop: be paranoid on exit and prevent new additions / removals
  * Bluetooth: add a mutex lock to avoid UAF in do_enale_set
  * soc: qcom: rpmh-rsc: Set suppress_bind_attrs flag
  * drm/tilcdc: fix leak & null ref in panel_connector_get_modes
  * ARM: socfpga: PM: add missing put_device() call in socfpga_setup_ocram_self_refresh()
  * spi: lantiq: fix: Rx overflow error in full duplex mode
  * ARM: at91: pm: add missing put_device() call in at91_pm_sram_init()
  * ARM: dts: gose: Fix ports node name for adv7612
  * ARM: dts: gose: Fix ports node name for adv7180
  * platform/x86: intel-vbtn: Fix return value check in check_acpi_dev()
  * platform/x86: intel-hid: Fix return value check in check_acpi_dev()
  * m68k: mac: Fix IOP status/control register writes
  * m68k: mac: Don't send IOP message until channel is idle
  * clk: scmi: Fix min and max rate when registering clocks with discrete rates
  * arm64: dts: exynos: Fix silent hang after boot on Espresso
  * firmware: arm_scmi: Fix SCMI genpd domain probing
  * crypto: ccree - fix resource leak on error path
  * arm64: dts: qcom: msm8916: Replace invalid bias-pull-none property
  * EDAC: Fix reference count leaks
  * arm64: dts: rockchip: fix rk3399-puma gmac reset gpio
  * arm64: dts: rockchip: fix rk3399-puma vcc5v0-host gpio
  * arm64: dts: rockchip: fix rk3368-lion gmac reset gpio
  * sched: correct SD_flags returned by tl->sd_flags()
  * sched/fair: Fix NOHZ next idle balance
  * x86/mce/inject: Fix a wrong assignment of i_mce.status
  * cgroup: add missing skcd->no_refcnt check in cgroup_sk_clone()
  * HID: input: Fix devices that return multiple bytes in battery report
  * tracepoint: Mark __tracepoint_string's __used
  * ANDROID: fix a bug in quota2
  * msm: ipa: set the mac header to 0 for rx skbs
  * ANDROID: Update the ABI xml based on the new driver core padding
  * ANDROID: GKI: add some padding to some driver core structures
  * ANDROID: GKI: Update the ABI xml representation
  * mhi: core: Free firmware memory at the end of mhi_fw_load_handler
  * backlight: qcom-spmi-wled: Force HFRC off when WLED is disabled
  * ANDROID: sched: add "frozen" field to task_struct
  * ANDROID: cgroups: add v2 freezer ABI changes
  * ANDROID: cgroups: ABI padding
  * Merge 4.19.139 into android-4.19-stable
  * Linux 4.19.139
  * Smack: fix use-after-free in smk_write_relabel_self()
  * i40e: Memory leak in i40e_config_iwarp_qvlist
  * i40e: Fix of memory leak and integer truncation in i40e_virtchnl.c
  * i40e: Wrong truncation from u16 to u8
  * i40e: add num_vectors checker in iwarp handler
  * rxrpc: Fix race between recvmsg and sendmsg on immediate call failure
  * selftests/net: relax cpu affinity requirement in msg_zerocopy test
  * Revert "vxlan: fix tos value before xmit"
  * openvswitch: Prevent kernel-infoleak in ovs_ct_put_key()
  * net: thunderx: use spin_lock_bh in nicvf_set_rx_mode_task()
  * net: gre: recompute gre csum for sctp over gre tunnels
  * hv_netvsc: do not use VF device if link is down
  * net: lan78xx: replace bogus endpoint lookup
  * vxlan: Ensure FDB dump is performed under RCU
  * net: ethernet: mtk_eth_soc: fix MTU warnings
  * ipv6: fix memory leaks on IPV6_ADDRFORM path
  * ipv4: Silence suspicious RCU usage warning
  * xattr: break delegations in {set,remove}xattr
  * Drivers: hv: vmbus: Ignore CHANNELMSG_TL_CONNECT_RESULT(23)
  * tools lib traceevent: Fix memory leak in process_dynamic_array_len
  * atm: fix atm_dev refcnt leaks in atmtcp_remove_persistent
  * igb: reinit_locked() should be called with rtnl_lock
  * cfg80211: check vendor command doit pointer before use
  * firmware: Fix a reference count leak.
  * usb: hso: check for return value in hso_serial_common_create()
  * i2c: slave: add sanity check when unregistering
  * i2c: slave: improve sanity check when registering
  * drm/nouveau/fbcon: zero-initialise the mode_cmd2 structure
  * drm/nouveau/fbcon: fix module unload when fbcon init has failed for some reason
  * net/9p: validate fds in p9_fd_open
  * leds: 88pm860x: fix use-after-free on unbind
  * leds: lm3533: fix use-after-free on unbind
  * leds: da903x: fix use-after-free on unbind
  * leds: wm831x-status: fix use-after-free on unbind
  * mtd: properly check all write ioctls for permissions
  * vgacon: Fix for missing check in scrollback handling
  * binder: Prevent context manager from incrementing ref 0
  * omapfb: dss: Fix max fclk divider for omap36xx
  * Bluetooth: Prevent out-of-bounds read in hci_inquiry_result_with_rssi_evt()
  * Bluetooth: Prevent out-of-bounds read in hci_inquiry_result_evt()
  * Bluetooth: Fix slab-out-of-bounds read in hci_extended_inquiry_result_evt()
  * staging: android: ashmem: Fix lockdep warning for write operation
  * ALSA: seq: oss: Serialize ioctls
  * Revert "ALSA: hda: call runtime_allow() for all hda controllers"
  * usb: xhci: Fix ASMedia ASM1142 DMA addressing
  * usb: xhci: define IDs for various ASMedia host controllers
  * USB: iowarrior: fix up report size handling for some devices
  * USB: serial: qcserial: add EM7305 QDL product ID
  * Revert "drm/dsi: Fix byte order of DCS set/get brightness"
  * BACKPORT: loop: Fix wrong masking of status flags
  * BACKPORT: loop: Add LOOP_CONFIGURE ioctl
  * BACKPORT: loop: Clean up LOOP_SET_STATUS lo_flags handling
  * BACKPORT: loop: Rework lo_ioctl() __user argument casting
  * BACKPORT: loop: Move loop_set_status_from_info() and friends up
  * BACKPORT: loop: Factor out configuring loop from status
  * BACKPORT: loop: Remove figure_loop_size()
  * BACKPORT: loop: Refactor loop_set_status() size calculation
  * BACKPORT: loop: Factor out setting loop device size
  * BACKPORT: loop: Remove sector_t truncation checks
  * BACKPORT: loop: Call loop_config_discard() only after new config is applied
  * Revert rpmh and usb changes
  * Merge 4.19.138 into android-4.19-stable
  * Linux 4.19.138
  * ext4: fix direct I/O read error
  * random32: move the pseudo-random 32-bit definitions to prandom.h
  * random32: remove net_rand_state from the latent entropy gcc plugin
  * random: fix circular include dependency on arm64 after addition of percpu.h
  * ARM: percpu.h: fix build error
  * random32: update the net random state on interrupt and activity
  * ANDROID: GKI: update the ABI xml
  * ANDROID: GKI: power: Add property to enable/disable cc toggle
  * ANDROID: Enforce KMI stability
  * Merge 4.19.137 into android-4.19-stable
  * Linux 4.19.137
  * x86/i8259: Use printk_deferred() to prevent deadlock
  * KVM: LAPIC: Prevent setting the tscdeadline timer if the lapic is hw disabled
  * xen-netfront: fix potential deadlock in xennet_remove()
  * cxgb4: add missing release on skb in uld_send()
  * x86/unwind/orc: Fix ORC for newly forked tasks
  * Revert "i2c: cadence: Fix the hold bit setting"
  * net: ethernet: ravb: exit if re-initialization fails in tx timeout
  * parisc: add support for cmpxchg on u8 pointers
  * nfc: s3fwrn5: add missing release on skb in s3fwrn5_recv_frame
  * qed: Disable "MFW indication via attention" SPAM every 5 minutes
  * usb: hso: Fix debug compile warning on sparc32
  * net/mlx5e: fix bpf_prog reference count leaks in mlx5e_alloc_rq
  * net: gemini: Fix missing clk_disable_unprepare() in error path of gemini_ethernet_port_probe()
  * Bluetooth: fix kernel oops in store_pending_adv_report
  * arm64: csum: Fix handling of bad packets
  * arm64/alternatives: move length validation inside the subsection
  * mac80211: mesh: Free pending skb when destroying a mpath
  * mac80211: mesh: Free ie data when leaving mesh
  * bpf: Fix map leak in HASH_OF_MAPS map
  * ibmvnic: Fix IRQ mapping disposal in error path
  * mlxsw: core: Free EMAD transactions using kfree_rcu()
  * mlxsw: core: Increase scope of RCU read-side critical section
  * mlx4: disable device on shutdown
  * net: lan78xx: fix transfer-buffer memory leak
  * net: lan78xx: add missing endpoint sanity check
  * net/mlx5: Verify Hardware supports requested ptp function on a given pin
  * sh: Fix validation of system call number
  * selftests/net: psock_fanout: fix clang issues for target arch PowerPC
  * selftests/net: rxtimestamp: fix clang issues for target arch PowerPC
  * xfrm: Fix crash when the hold queue is used.
  * net/x25: Fix null-ptr-deref in x25_disconnect
  * net/x25: Fix x25_neigh refcnt leak when x25 disconnect
  * xfs: fix missed wakeup on l_flush_wait
  * rds: Prevent kernel-infoleak in rds_notify_queue_get()
  * drm: hold gem reference until object is no longer accessed
  * drm/amdgpu: Prevent kernel-infoleak in amdgpu_info_ioctl()
  * Revert "drm/amdgpu: Fix NULL dereference in dpm sysfs handlers"
  * ARM: 8986/1: hw_breakpoint: Don't invoke overflow handler on uaccess watchpoints
  * wireless: Use offsetof instead of custom macro.
  * 9p/trans_fd: Fix concurrency del of req_list in p9_fd_cancelled/p9_read_work
  * PCI/ASPM: Disable ASPM on ASMedia ASM1083/1085 PCIe-to-PCI bridge
  * Btrfs: fix selftests failure due to uninitialized i_mode in test inodes
  * sctp: implement memory accounting on tx path
  * btrfs: inode: Verify inode mode to avoid NULL pointer dereference
  * drm/amd/display: prevent memory leak
  * ath9k: release allocated buffer if timed out
  * ath9k_htc: release allocated buffer if timed out
  * tracing: Have error path in predicate_parse() free its allocated memory
  * drm/amdgpu: fix multiple memory leaks in acp_hw_init
  * iio: imu: adis16400: fix memory leak
  * media: rc: prevent memory leak in cx23888_ir_probe
  * crypto: ccp - Release all allocated memory if sha type is invalid
  * f2fs: prepare a waiter before entering io_schedule
  * f2fs: update_sit_entry: Make the judgment condition of f2fs_bug_on more intuitive
  * f2fs: replace test_and_set/clear_bit() with set/clear_bit()
  * f2fs: make file immutable even if releasing zero compression block
  * f2fs: compress: disable compression mount option if compression is off
  * msm: ipa3: Schedule the NAPI only from interrupt context
  * ANDROID: GKI: kernel: tick-sched: Move wake callback registration code
  * Merge 4.19.136 into android-4.19-stable
  * Linux 4.19.136
  * regmap: debugfs: check count when read regmap file
  * rtnetlink: Fix memory(net_device) leak when ->newlink fails
  * udp: Improve load balancing for SO_REUSEPORT.
  * udp: Copy has_conns in reuseport_grow().
  * sctp: shrink stream outq when fails to do addstream reconf
  * sctp: shrink stream outq only when new outcnt < old outcnt
  * AX.25: Prevent integer overflows in connect and sendmsg
  * tcp: allow at most one TLP probe per flight
  * rxrpc: Fix sendmsg() returning EPIPE due to recvmsg() returning ENODATA
  * qrtr: orphan socket in qrtr_release()
  * net: udp: Fix wrong clean up for IS_UDPLITE macro
  * net-sysfs: add a newline when printing 'tx_timeout' by sysfs
  * ip6_gre: fix null-ptr-deref in ip6gre_init_net()
  * drivers/net/wan/x25_asy: Fix to make it work
  * dev: Defer free of skbs in flush_backlog
  * AX.25: Prevent out-of-bounds read in ax25_sendmsg()
  * AX.25: Fix out-of-bounds read in ax25_connect()
  * f2fs: compress: add sanity check during compressed cluster read
  * f2fs: use macro instead of f2fs verity version
  * f2fs: fix deadlock between quota writes and checkpoint
  * Merge 4.19.135 into android-4.19-stable
  * Linux 4.19.135
  * ath9k: Fix regression with Atheros 9271
  * ath9k: Fix general protection fault in ath9k_hif_usb_rx_cb
  * dm integrity: fix integrity recalculation that is improperly skipped
  * ASoC: qcom: Drop HAS_DMA dependency to fix link failure
  * ASoC: rt5670: Add new gpio1_is_ext_spk_en quirk and enable it on the Lenovo Miix 2 10
  * x86, vmlinux.lds: Page-align end of ..page_aligned sections
  * parisc: Add atomic64_set_release() define to avoid CPU soft lockups
  * drm/amd/powerplay: fix a crash when overclocking Vega M
  * drm/amdgpu: Fix NULL dereference in dpm sysfs handlers
  * io-mapping: indicate mapping failure
  * mm: memcg/slab: fix memory leak at non-root kmem_cache destroy
  * mm: memcg/slab: synchronize access to kmem_cache dying flag using a spinlock
  * mm/memcg: fix refcount error while moving and swapping
  * Makefile: Fix GCC_TOOLCHAIN_DIR prefix for Clang cross compilation
  * vt: Reject zero-sized screen buffer size.
  * fbdev: Detect integer underflow at "struct fbcon_ops"->clear_margins.
  * serial: 8250_mtk: Fix high-speed baud rates clamping
  * serial: 8250: fix null-ptr-deref in serial8250_start_tx()
  * staging: comedi: addi_apci_1564: check INSN_CONFIG_DIGITAL_TRIG shift
  * staging: comedi: addi_apci_1500: check INSN_CONFIG_DIGITAL_TRIG shift
  * staging: comedi: ni_6527: fix INSN_CONFIG_DIGITAL_TRIG support
  * staging: comedi: addi_apci_1032: check INSN_CONFIG_DIGITAL_TRIG shift
  * staging: wlan-ng: properly check endpoint types
  * Revert "cifs: Fix the target file was deleted when rename failed."
  * usb: xhci: Fix ASM2142/ASM3142 DMA addressing
  * usb: xhci-mtk: fix the failure of bandwidth allocation
  * binder: Don't use mmput() from shrinker function.
  * RISC-V: Upgrade smp_mb__after_spinlock() to iorw,iorw
  * x86: math-emu: Fix up 'cmp' insn for clang ias
  * arm64: Use test_tsk_thread_flag() for checking TIF_SINGLESTEP
  * hwmon: (scmi) Fix potential buffer overflow in scmi_hwmon_probe()
  * hwmon: (adm1275) Make sure we are reading enough data for different chips
  * usb: gadget: udc: gr_udc: fix memleak on error handling path in gr_ep_init()
  * Input: synaptics - enable InterTouch for ThinkPad X1E 1st gen
  * dmaengine: ioat setting ioat timeout as module parameter
  * hwmon: (aspeed-pwm-tacho) Avoid possible buffer overflow
  * regmap: dev_get_regmap_match(): fix string comparison
  * spi: mediatek: use correct SPI_CFG2_REG MACRO
  * Input: add `SW_MACHINE_COVER`
  * dmaengine: tegra210-adma: Fix runtime PM imbalance on error
  * HID: apple: Disable Fn-key key-re-mapping on clone keyboards
  * HID: steam: fixes race in handling device list.
  * HID: alps: support devices with report id 2
  * HID: i2c-hid: add Mediacom FlexBook edge13 to descriptor override
  * scripts/gdb: fix lx-symbols 'gdb.error' while loading modules
  * scripts/decode_stacktrace: strip basepath from all paths
  * serial: exar: Fix GPIO configuration for Sealevel cards based on XR17V35X
  * bonding: check return value of register_netdevice() in bond_newlink()
  * i2c: rcar: always clear ICSAR to avoid side effects
  * net: ethernet: ave: Fix error returns in ave_init
  * ipvs: fix the connection sync failed in some cases
  * qed: suppress "don't support RoCE & iWARP" flooding on HW init
  * mlxsw: destroy workqueue when trap_register in mlxsw_emad_init
  * bonding: check error value of register_netdevice() immediately
  * net: smc91x: Fix possible memory leak in smc_drv_probe()
  * drm: sun4i: hdmi: Fix inverted HPD result
  * ieee802154: fix one possible memleak in adf7242_probe
  * net: dp83640: fix SIOCSHWTSTAMP to update the struct with actual configuration
  * ax88172a: fix ax88172a_unbind() failures
  * hippi: Fix a size used in a 'pci_free_consistent()' in an error handling path
  * fpga: dfl: fix bug in port reset handshake
  * bnxt_en: Fix race when modifying pause settings.
  * btrfs: fix page leaks after failure to lock page for delalloc
  * btrfs: fix mount failure caused by race with umount
  * btrfs: fix double free on ulist after backref resolution failure
  * ASoC: rt5670: Correct RT5670_LDO_SEL_MASK
  * ALSA: info: Drop WARN_ON() from buffer NULL sanity check
  * uprobes: Change handle_swbp() to send SIGTRAP with si_code=SI_KERNEL, to fix GDB regression
  * IB/umem: fix reference count leak in ib_umem_odp_get()
  * tipc: clean up skb list lock handling on send path
  * spi: spi-fsl-dspi: Exit the ISR with IRQ_NONE when it's not ours
  * SUNRPC reverting d03727b248d0 ("NFSv4 fix CLOSE not waiting for direct IO compeletion")
  * irqdomain/treewide: Keep firmware node unconditionally allocated
  * fuse: fix weird page warning
  * drivers/firmware/psci: Fix memory leakage in alloc_init_cpu_groups()
  * drm/nouveau/i2c/g94-: increase NV_PMGR_DP_AUXCTL_TRANSACTREQ timeout
  * net: sky2: initialize return of gm_phy_read
  * drivers/net/wan/lapbether: Fixed the value of hard_header_len
  * xtensa: update *pos in cpuinfo_op.next
  * xtensa: fix __sync_fetch_and_{and,or}_4 declarations
  * scsi: scsi_transport_spi: Fix function pointer check
  * mac80211: allow rx of mesh eapol frames with default rx key
  * pinctrl: amd: fix npins for uart0 in kerncz_groups
  * gpio: arizona: put pm_runtime in case of failure
  * gpio: arizona: handle pm_runtime_get_sync failure case
  * soc: qcom: rpmh: Dirt can only make you dirtier, not cleaner
  * Reverting below patches from android-4.19-stable.125
  * ANDROID: build: update ABI definitions
  * f2fs: correct comment of f2fs_exist_written_data
  * ANDROID: update the kernel release format for GKI
  * f2fs: compress: delay temp page allocation
  * f2fs: compress: fix to update isize when overwriting compressed file
  * f2fs: space related cleanup
  * f2fs: fix use-after-free issue
  * f2fs: Change the type of f2fs_flush_inline_data() to void
  * f2fs: add F2FS_IOC_SEC_TRIM_FILE ioctl
  * f2fs: should avoid inode eviction in synchronous path
  * f2fs: segment.h: delete a duplicated word
  * f2fs: compress: fix to avoid memory leak on cc->cpages
  * ANDROID: Incremental fs: magic number compatible 32-bit
  * ANDROID: kbuild: don't merge .*..compoundliteral in modules
  * ANDROID: GKI: preserve ABI for struct sock_cgroup_data
  * Revert "genetlink: remove genl_bind"
  * Revert "arm64/alternatives: use subsections for replacement sequences"
  * Merge 4.19.134 into android-4.19-stable
  * Linux 4.19.134
  * spi: sprd: switch the sequence of setting WDG_LOAD_LOW and _HIGH
  * rxrpc: Fix trace string
  * libceph: don't omit recovery_deletes in target_copy()
  * printk: queue wake_up_klogd irq_work only if per-CPU areas are ready
  * genirq/affinity: Handle affinity setting on inactive interrupts correctly
  * sched/fair: handle case of task_h_load() returning 0
  * sched: Fix unreliable rseq cpu_id for new tasks
  * arm64: compat: Ensure upper 32 bits of x0 are zero on syscall return
  * arm64: ptrace: Consistently use pseudo-singlestep exceptions
  * arm64: ptrace: Override SPSR.SS when single-stepping is enabled
  * thermal/drivers/cpufreq_cooling: Fix wrong frequency converted from power
  * misc: atmel-ssc: lock with mutex instead of spinlock
  * dmaengine: fsl-edma: Fix NULL pointer exception in fsl_edma_tx_handler
  * intel_th: Fix a NULL dereference when hub driver is not loaded
  * intel_th: pci: Add Emmitsburg PCH support
  * intel_th: pci: Add Tiger Lake PCH-H support
  * intel_th: pci: Add Jasper Lake CPU support
  * powerpc/book3s64/pkeys: Fix pkey_access_permitted() for execute disable pkey
  * hwmon: (emc2103) fix unable to change fan pwm1_enable attribute
  * riscv: use 16KB kernel stack on 64-bit
  * MIPS: Fix build for LTS kernel caused by backporting lpj adjustment
  * timer: Fix wheel index calculation on last level
  * timer: Prevent base->clk from moving backward
  * uio_pdrv_genirq: fix use without device tree and no interrupt
  * Input: i8042 - add Lenovo XiaoXin Air 12 to i8042 nomux list
  * mei: bus: don't clean driver pointer
  * Revert "zram: convert remaining CLASS_ATTR() to CLASS_ATTR_RO()"
  * fuse: Fix parameter for FS_IOC_{GET,SET}FLAGS
  * ovl: fix unneeded call to ovl_change_flags()
  * ovl: relax WARN_ON() when decoding lower directory file handle
  * ovl: inode reference leak in ovl_is_inuse true case.
  * serial: mxs-auart: add missed iounmap() in probe failure and remove
  * virtio: virtio_console: add missing MODULE_DEVICE_TABLE() for rproc serial
  * virt: vbox: Fix guest capabilities mask check
  * virt: vbox: Fix VBGL_IOCTL_VMMDEV_REQUEST_BIG and _LOG req numbers to match upstream
  * USB: serial: option: add Quectel EG95 LTE modem
  * USB: serial: option: add GosunCn GM500 series
  * USB: serial: ch341: add new Product ID for CH340
  * USB: serial: cypress_m8: enable Simply Automated UPB PIM
  * USB: serial: iuu_phoenix: fix memory corruption
  * usb: gadget: function: fix missing spinlock in f_uac1_legacy
  * usb: chipidea: core: add wakeup support for extcon
  * usb: dwc2: Fix shutdown callback in platform
  * USB: c67x00: fix use after free in c67x00_giveback_urb
  * ALSA: hda/realtek - Enable Speaker for ASUS UX533 and UX534
  * ALSA: hda/realtek - change to suitable link model for ASUS platform
  * ALSA: usb-audio: Fix race against the error recovery URB submission
  * ALSA: line6: Sync the pending work cancel at disconnection
  * ALSA: line6: Perform sanity check for each URB creation
  * HID: quirks: Ignore Simply Automated UPB PIM
  * HID: quirks: Always poll Obins Anne Pro 2 keyboard
  * HID: magicmouse: do not set up autorepeat
  * slimbus: core: Fix mismatch in of_node_get/put
  * mtd: rawnand: oxnas: Release all devices in the _remove() path
  * mtd: rawnand: oxnas: Unregister all devices on error
  * mtd: rawnand: oxnas: Keep track of registered devices
  * mtd: rawnand: brcmnand: fix CS0 layout
  * mtd: rawnand: timings: Fix default tR_max and tCCS_min timings
  * mtd: rawnand: marvell: Fix probe error path
  * mtd: rawnand: marvell: Use nand_cleanup() when the device is not yet registered
  * soc: qcom: rpmh-rsc: Allow using free WAKE TCS for active request
  * soc: qcom: rpmh-rsc: Clear active mode configuration for wake TCS
  * soc: qcom: rpmh: Invalidate SLEEP and WAKE TCSes before flushing new data
  * soc: qcom: rpmh: Update dirty flag only when data changes
  * perf stat: Zero all the 'ena' and 'run' array slot stats for interval mode
  * apparmor: ensure that dfa state tables have entries
  * copy_xstate_to_kernel: Fix typo which caused GDB regression
  * regmap: debugfs: Don't sleep while atomic for fast_io regmaps
  * ARM: dts: socfpga: Align L2 cache-controller nodename with dtschema
  * Revert "thermal: mediatek: fix register index error"
  * staging: comedi: verify array index is correct before using it
  * usb: gadget: udc: atmel: fix uninitialized read in debug printk
  * spi: spi-sun6i: sun6i_spi_transfer_one(): fix setting of clock rate
  * arm64: dts: meson: add missing gxl rng clock
  * phy: sun4i-usb: fix dereference of pointer phy0 before it is null checked
  * iio:health:afe4404 Fix timestamp alignment and prevent data leak.
  * ALSA: usb-audio: Add registration quirk for Kingston HyperX Cloud Flight S
  * ACPI: video: Use native backlight on Acer TravelMate 5735Z
  * Input: mms114 - add extra compatible for mms345l
  * ALSA: usb-audio: Add registration quirk for Kingston HyperX Cloud Alpha S
  * ACPI: video: Use native backlight on Acer Aspire 5783z
  * ALSA: usb-audio: Rewrite registration quirk handling
  * mmc: sdhci: do not enable card detect interrupt for gpio cd type
  * doc: dt: bindings: usb: dwc3: Update entries for disabling SS instances in park mode
  * ALSA: usb-audio: Create a registration quirk for Kingston HyperX Amp (0951:16d8)
  * scsi: sr: remove references to BLK_DEV_SR_VENDOR, leave it enabled
  * ARM: at91: pm: add quirk for sam9x60's ulp1
  * HID: quirks: Remove ITE 8595 entry from hid_have_special_driver
  * net: sfp: add some quirks for GPON modules
  * net: sfp: add support for module quirks
  * Revert "usb/ehci-platform: Set PM runtime as active on resume"
  * Revert "usb/xhci-plat: Set PM runtime as active on resume"
  * Revert "usb/ohci-platform: Fix a warning when hibernating"
  * of: of_mdio: Correct loop scanning logic
  * net: dsa: bcm_sf2: Fix node reference count
  * spi: spi-fsl-dspi: Fix lockup if device is shutdown during SPI transfer
  * spi: fix initial SPI_SR value in spi-fsl-dspi
  * iio:health:afe4403 Fix timestamp alignment and prevent data leak.
  * iio:pressure:ms5611 Fix buffer element alignment
  * iio:humidity:hts221 Fix alignment and data leak issues
  * iio: pressure: zpa2326: handle pm_runtime_get_sync failure
  * iio: mma8452: Add missed iio_device_unregister() call in mma8452_probe()
  * iio: magnetometer: ak8974: Fix runtime PM imbalance on error
  * iio:humidity:hdc100x Fix alignment and data leak issues
  * iio:magnetometer:ak8974: Fix alignment and data leak issues
  * arm64/alternatives: don't patch up internal branches
  * i2c: eg20t: Load module automatically if ID matches
  * gfs2: read-only mounts should grab the sd_freeze_gl glock
  * tpm_tis: extra chip->ops check on error path in tpm_tis_core_init
  * arm64/alternatives: use subsections for replacement sequences
  * m68k: mm: fix node memblock init
  * m68k: nommu: register start of the memory with memblock
  * drm/exynos: fix ref count leak in mic_pre_enable
  * drm/msm: fix potential memleak in error branch
  * vlan: consolidate VLAN parsing code and limit max parsing depth
  * sched: consistently handle layer3 header accesses in the presence of VLANs
  * cgroup: Fix sock_cgroup_data on big-endian.
  * cgroup: fix cgroup_sk_alloc() for sk_clone_lock()
  * tcp: md5: allow changing MD5 keys in all socket states
  * tcp: md5: refine tcp_md5_do_add()/tcp_md5_hash_key() barriers
  * tcp: md5: do not send silly options in SYNCOOKIES
  * tcp: md5: add missing memory barriers in tcp_md5_do_add()/tcp_md5_hash_key()
  * tcp: make sure listeners don't initialize congestion-control state
  * tcp: fix SO_RCVLOWAT possible hangs under high mem pressure
  * net: usb: qmi_wwan: add support for Quectel EG95 LTE modem
  * net_sched: fix a memory leak in atm_tc_init()
  * net: Added pointer check for dst->ops->neigh_lookup in dst_neigh_lookup_skb
  * llc: make sure applications use ARPHRD_ETHER
  * l2tp: remove skb_dst_set() from l2tp_xmit_skb()
  * ipv4: fill fl4_icmp_{type,code} in ping_v4_sendmsg
  * genetlink: remove genl_bind
  * net: rmnet: fix lower interface leak
  * perf: Make perf able to build with latest libbfd
  * UPSTREAM: media: v4l2-ctrl: Add H264 profile and levels
  * UPSTREAM: media: v4l2-ctrl: Add control for h.264 chroma qp offset
  * ANDROID: GKI: ASoC: compress: revert some code to avoid race condition
  * ANDROID: GKI: Update the ABI xml representation.
  * ANDROID: GKI: kernel: tick-sched: Add an API for wakeup callbacks
  * ANDROID: ASoC: Compress: Check and set pcm_new driver op
  * Revert "ANDROID: GKI: arm64: gki_defconfig: Disable CONFIG_ARM64_TAGGED_ADDR_ABI"
  * ANDROID: arm64: configs: enabe CONFIG_TMPFS
  * Revert "ALSA: compress: fix partial_drain completion state"
  * Merge 4.19.133 into android-4.19-stable
  * ANDROID: GKI: enable CONFIG_EXT4_FS_POSIX_ACL.
  * ANDROID: GKI: set CONFIG_STATIC_USERMODEHELPER_PATH
  * f2fs: use generic names for generic ioctls
  * Linux 4.19.133
  * s390/mm: fix huge pte soft dirty copying
  * ARC: elf: use right ELF_ARCH
  * ARC: entry: fix potential EFA clobber when TIF_SYSCALL_TRACE
  * dm: use noio when sending kobject event
  * drm/radeon: fix double free
  * btrfs: fix fatal extent_buffer readahead vs releasepage race
  * Revert "ath9k: Fix general protection fault in ath9k_hif_usb_rx_cb"
  * bpf: Check correct cred for CAP_SYSLOG in bpf_dump_raw_ok()
  * kprobes: Do not expose probe addresses to non-CAP_SYSLOG
  * module: Do not expose section addresses to non-CAP_SYSLOG
  * module: Refactor section attr into bin attribute
  * kernel: module: Use struct_size() helper
  * kallsyms: Refactor kallsyms_show_value() to take cred
  * KVM: x86: Mark CR4.TSD as being possibly owned by the guest
  * KVM: x86: Inject #GP if guest attempts to toggle CR4.LA57 in 64-bit mode
  * KVM: x86: bit 8 of non-leaf PDPEs is not reserved
  * KVM: arm64: Stop clobbering x0 for HVC_SOFT_RESTART
  * KVM: arm64: Fix definition of PAGE_HYP_DEVICE
  * ALSA: usb-audio: add quirk for MacroSilicon MS2109
  * ALSA: hda - let hs_mic be picked ahead of hp_mic
  * ALSA: opl3: fix infoleak in opl3
  * mlxsw: spectrum_router: Remove inappropriate usage of WARN_ON()
  * net: macb: mark device wake capable when "magic-packet" property present
  * bnxt_en: fix NULL dereference in case SR-IOV configuration fails
  * cxgb4: fix all-mask IP address comparison
  * nbd: Fix memory leak in nbd_add_socket
  * arm64: kgdb: Fix single-step exception handling oops
  * ALSA: compress: fix partial_drain completion state
  * net: hns3: fix use-after-free when doing self test
  * smsc95xx: avoid memory leak in smsc95xx_bind
  * smsc95xx: check return value of smsc95xx_reset
  * net: cxgb4: fix return error value in t4_prep_fw
  * drm/mediatek: Check plane visibility in atomic_update
  * net: qrtr: Fix an out of bounds read qrtr_endpoint_post()
  * x86/entry: Increase entry_stack size to a full page
  * nvme-rdma: assign completion vector correctly
  * block: release bip in a right way in error path
  * usb: dwc3: pci: Fix reference count leak in dwc3_pci_resume_work
  * scsi: mptscsih: Fix read sense data size
  * ARM: imx6: add missing put_device() call in imx6q_suspend_init()
  * cifs: update ctime and mtime during truncate
  * s390/kasan: fix early pgm check handler execution
  * drm: panel-orientation-quirks: Use generic orientation-data for Acer S1003
  * drm: panel-orientation-quirks: Add quirk for Asus T101HA panel
  * i40e: protect ring accesses with READ- and WRITE_ONCE
  * ixgbe: protect ring accesses with READ- and WRITE_ONCE
  * spi: spidev: fix a potential use-after-free in spidev_release()
  * spi: spidev: fix a race between spidev_release and spidev_remove
  * gpu: host1x: Detach driver on unregister
  * drm/tegra: hub: Do not enable orphaned window group
  * ARM: dts: omap4-droid4: Fix spi configuration and increase rate
  * regmap: fix alignment issue
  * spi: spi-fsl-dspi: Fix external abort on interrupt in resume or exit paths
  * spi: spi-fsl-dspi: use IRQF_SHARED mode to request IRQ
  * spi: spi-fsl-dspi: Fix lockup if device is removed during SPI transfer
  * spi: spi-fsl-dspi: Adding shutdown hook
  * KVM: s390: reduce number of IO pins to 1
  * ANDROID: GKI: update abi based on padding fields being added
  * ANDROID: GKI: USB: Gadget: add Android ABI padding to struct usb_gadget
  * ANDROID: GKI: sound/usb/card.h: add Android ABI padding to struct snd_usb_endpoint
  * ANDROID: fscrypt: fix DUN contiguity with inline encryption + IV_INO_LBLK_32 policies
  * cnss2: Add support to use new ramdump APIs
  * ANDROID: f2fs: add back compress inode check
  * Merge 4.19.132 into android-4.19-stable
  * Linux 4.19.132
  * efi: Make it possible to disable efivar_ssdt entirely
  * dm zoned: assign max_io_len correctly
  * irqchip/gic: Atomically update affinity
  * MIPS: Add missing EHB in mtc0 -> mfc0 sequence for DSPen
  * cifs: Fix the target file was deleted when rename failed.
  * SMB3: Honor lease disabling for multiuser mounts
  * SMB3: Honor persistent/resilient handle flags for multiuser mounts
  * SMB3: Honor 'seal' flag for multiuser mounts
  * Revert "ALSA: usb-audio: Improve frames size computation"
  * nfsd: apply umask on fs without ACL support
  * i2c: mlxcpld: check correct size of maximum RECV_LEN packet
  * i2c: algo-pca: Add 0x78 as SCL stuck low status for PCA9665
  * nvme: fix a crash in nvme_mpath_add_disk
  * SMB3: Honor 'posix' flag for multiuser mounts
  * virtio-blk: free vblk-vqs in error path of virtblk_probe()
  * drm: sun4i: hdmi: Remove extra HPD polling
  * hwmon: (acpi_power_meter) Fix potential memory leak in acpi_power_meter_add()
  * hwmon: (max6697) Make sure the OVERT mask is set correctly
  * cxgb4: fix SGE queue dump destination buffer context
  * cxgb4: use correct type for all-mask IP address comparison
  * cxgb4: parse TC-U32 key values and masks natively
  * cxgb4: use unaligned conversion for fetching timestamp
  * drm/msm/dpu: fix error return code in dpu_encoder_init
  * crypto: af_alg - fix use-after-free in af_alg_accept() due to bh_lock_sock()
  * kgdb: Avoid suspicious RCU usage warning
  * nvme-multipath: fix deadlock between ana_work and scan_work
  * nvme-multipath: set bdi capabilities once
  * s390/debug: avoid kernel warning on too large number of pages
  * usb: usbtest: fix missing kfree(dev->buf) in usbtest_disconnect
  * mm/slub: fix stack overruns with SLUB_STATS
  * mm/slub.c: fix corrupted freechain in deactivate_slab()
  * usbnet: smsc95xx: Fix use-after-free after removal
  * EDAC/amd64: Read back the scrub rate PCI register on F15h
  * mm: fix swap cache node allocation mask
  * btrfs: fix a block group ref counter leak after failure to remove block group
  * f2fs: don't keep meta inode pages used for compressed block migration
  * f2fs: fix error path in do_recover_data()
  * f2fs: fix to wait GCed compressed page writeback
  * f2fs: remove write attribute of main_blkaddr sysfs node
  * ANDROID: Update ABI representation for libabigail update
  * ANDROID: Update the ABI representation
  * f2fs: add GC_URGENT_LOW mode in gc_urgent
  * f2fs: avoid readahead race condition
  * f2fs: fix return value of move_data_block()
  * f2fs: add parameter op_flag in f2fs_submit_page_read()
  * f2fs: split f2fs_allocate_new_segments()
  * f2fs: lost matching-pair of trace in f2fs_truncate_inode_blocks
  * f2fs: fix an oops in f2fs_is_compressed_page
  * f2fs: make trace enter and end in pairs for unlink
  * f2fs: fix to check page dirty status before writeback
  * f2fs: remove the unused compr parameter
  * f2fs: support to trace f2fs_fiemap()
  * f2fs: support to trace f2fs_bmap()
  * f2fs: fix wrong return value of f2fs_bmap_compress()
  * f2fs: remove useless parameter of __insert_free_nid()
  * f2fs: fix typo in comment of f2fs_do_add_link
  * f2fs: fix to wait page writeback before update
  * f2fs: show more debug info for per-temperature log
  * f2fs: add f2fs_gc exception handle in f2fs_ioc_gc_range
  * f2fs: clean up parameter of f2fs_allocate_data_block()
  * f2fs: shrink node_write lock coverage
  * f2fs: add prefix for exported symbols
  * ANDROID: Update the ABI xml representation
  * ANDROID: GKI: fix ABI diffs caused by GPU heap and pool vmstat additions
  * ANDROID: sched: consider stune boost margin when computing energy
  * ANDROID: GKI: move abi files to android/
  * ANDROID: GKI: drop unneeded "_whitelist" off of symbol filenames
  * UPSTREAM: binder: fix null deref of proc->context
  * ANDROID: cpufreq: schedutil: maintain raw cache when next_f is not changed
  * UPSTREAM: net: bpf: Make bpf_ktime_get_ns() available to non GPL programs
  * UPSTREAM: usb: musb: mediatek: add reset FADDR to zero in reset interrupt handle
  * ANDROID: GKI: scripts: Makefile: update the lz4 command (#2)
  * ANDROID: Update the ABI xml representation
  * Revert "drm/dsi: Fix byte order of DCS set/get brightness"
  * Merge 4.19.131 into android-4.19-stable
  * Linux 4.19.131
  * Revert "tty: hvc: Fix data abort due to race in hvc_open"
  * xfs: add agf freeblocks verify in xfs_agf_verify
  * dm writecache: add cond_resched to loop in persistent_memory_claim()
  * dm writecache: correct uncommitted_block when discarding uncommitted entry
  * NFSv4 fix CLOSE not waiting for direct IO compeletion
  * pNFS/flexfiles: Fix list corruption if the mirror count changes
  * SUNRPC: Properly set the @subbuf parameter of xdr_buf_subsegment()
  * sunrpc: fixed rollback in rpc_gssd_dummy_populate()
  * Staging: rtl8723bs: prevent buffer overflow in update_sta_support_rate()
  * drm/radeon: fix fb_div check in ni_init_smc_spll_table()
  * drm: rcar-du: Fix build error
  * ring-buffer: Zero out time extend if it is nested and not absolute
  * tracing: Fix event trigger to accept redundant spaces
  * arm64: perf: Report the PC value in REGS_ABI_32 mode
  * ocfs2: fix panic on nfs server over ocfs2
  * ocfs2: fix value of OCFS2_INVALID_SLOT
  * ocfs2: load global_inode_alloc
  * ocfs2: avoid inode removal while nfsd is accessing it
  * mm/slab: use memzero_explicit() in kzfree()
  * btrfs: fix failure of RWF_NOWAIT write into prealloc extent beyond eof
  * btrfs: fix data block group relocation failure due to concurrent scrub
  * x86/asm/64: Align start of __clear_user() loop to 16-bytes
  * KVM: nVMX: Plumb L2 GPA through to PML emulation
  * KVM: X86: Fix MSR range of APIC registers in X2APIC mode
  * erofs: fix partially uninitialized misuse in z_erofs_onlinepage_fixup
  * ACPI: sysfs: Fix pm_profile_attr type
  * ALSA: hda/realtek - Add quirk for MSI GE63 laptop
  * ALSA: hda: Add NVIDIA codec IDs 9a & 9d through a0 to patch table
  * RISC-V: Don't allow write+exec only page mapping request in mmap
  * blktrace: break out of blktrace setup on concurrent calls
  * kbuild: improve cc-option to clean up all temporary files
  * arm64: sve: Fix build failure when ARM64_SVE=y and SYSCTL=n
  * s390/vdso: fix vDSO clock_getres()
  * s390/ptrace: fix setting syscall number
  * net: alx: fix race condition in alx_remove
  * ibmvnic: Harden device login requests
  * hwrng: ks-sa - Fix runtime PM imbalance on error
  * riscv/atomic: Fix sign extension for RV64I
  * drm/amd/display: Use kfree() to free rgb_user in calculate_user_regamma_ramp()
  * ata/libata: Fix usage of page address by page_address in ata_scsi_mode_select_xlat function
  * sata_rcar: handle pm_runtime_get_sync failure cases
  * sched/core: Fix PI boosting between RT and DEADLINE tasks
  * sched/deadline: Initialize ->dl_boosted
  * i2c: core: check returned size of emulated smbus block read
  * i2c: fsi: Fix the port number field in status register
  * net: bcmgenet: use hardware padding of runt frames
  * netfilter: ipset: fix unaligned atomic access
  * usb: gadget: udc: Potential Oops in error handling code
  * ARM: imx5: add missing put_device() call in imx_suspend_alloc_ocram()
  * cxgb4: move handling L2T ARP failures to caller
  * net: qed: fix excessive QM ILT lines consumption
  * net: qed: fix NVMe login fails over VFs
  * net: qed: fix left elements count calculation
  * RDMA/mad: Fix possible memory leak in ib_mad_post_receive_mads()
  * ASoC: rockchip: Fix a reference count leak.
  * RDMA/cma: Protect bind_list and listen_list while finding matching cm id
  * RDMA/qedr: Fix KASAN: use-after-free in ucma_event_handler+0x532
  * rxrpc: Fix handling of rwind from an ACK packet
  * ARM: dts: NSP: Correct FA2 mailbox node
  * regmap: Fix memory leak from regmap_register_patch
  * x86/resctrl: Fix a NULL vs IS_ERR() static checker warning in rdt_cdp_peer_get()
  * ARM: dts: Fix duovero smsc interrupt for suspend
  * ASoC: fsl_ssi: Fix bclk calculation for mono channel
  * regualtor: pfuze100: correct sw1a/sw2 on pfuze3000
  * efi/esrt: Fix reference count leak in esre_create_sysfs_entry.
  * ASoC: q6asm: handle EOS correctly
  * xfrm: Fix double ESP trailer insertion in IPsec crypto offload.
  * cifs/smb3: Fix data inconsistent when zero file range
  * cifs/smb3: Fix data inconsistent when punch hole
  * IB/mad: Fix use after free when destroying MAD agent
  * loop: replace kill_bdev with invalidate_bdev
  * cdc-acm: Add DISABLE_ECHO quirk for Microchip/SMSC chip
  * xhci: Return if xHCI doesn't support LPM
  * xhci: Fix enumeration issue when setting max packet size for FS devices.
  * xhci: Fix incorrect EP_STATE_MASK
  * scsi: zfcp: Fix panic on ERP timeout for previously dismissed ERP action
  * ALSA: usb-audio: Fix OOB access of mixer element list
  * ALSA: usb-audio: add quirk for Samsung USBC Headset (AKG)
  * ALSA: usb-audio: add quirk for Denon DCD-1500RE
  * usb: typec: tcpci_rt1711h: avoid screaming irq causing boot hangs
  * usb: host: ehci-exynos: Fix error check in exynos_ehci_probe()
  * xhci: Poll for U0 after disabling USB2 LPM
  * usb: host: xhci-mtk: avoid runtime suspend when removing hcd
  * USB: ehci: reopen solution for Synopsys HC bug
  * usb: add USB_QUIRK_DELAY_INIT for Logitech C922
  * usb: dwc2: Postponed gadget registration to the udc class driver
  * USB: ohci-sm501: Add missed iounmap() in remove
  * net: core: reduce recursion limit value
  * net: Do not clear the sock TX queue in sk_set_socket()
  * net: Fix the arp error in some cases
  * sch_cake: don't call diffserv parsing code when it is not needed
  * tcp_cubic: fix spurious HYSTART_DELAY exit upon drop in min RTT
  * sch_cake: fix a few style nits
  * sch_cake: don't try to reallocate or unshare skb unconditionally
  * ip_tunnel: fix use-after-free in ip_tunnel_lookup()
  * net: phy: Check harder for errors in get_phy_id()
  * ip6_gre: fix use-after-free in ip6gre_tunnel_lookup()
  * tg3: driver sleeps indefinitely when EEH errors exceed eeh_max_freezes
  * tcp: grow window for OOO packets only for SACK flows
  * tcp: don't ignore ECN CWR on pure ACK
  * sctp: Don't advertise IPv4 addresses if ipv6only is set on the socket
  * rxrpc: Fix notification call on completion of discarded calls
  * rocker: fix incorrect error handling in dma_rings_init
  * net: usb: ax88179_178a: fix packet alignment padding
  * net: increment xmit_recursion level in dev_direct_xmit()
  * net: use correct this_cpu primitive in dev_recursion_level
  * net: place xmit recursion in softnet data
  * net: fix memleak in register_netdevice()
  * net: bridge: enfore alignment for ethernet address
  * mld: fix memory leak in ipv6_mc_destroy_dev()
  * ibmveth: Fix max MTU limit
  * apparmor: don't try to replace stale label in ptraceme check
  * ALSA: hda/realtek - Enable micmute LED on and HP system
  * ALSA: hda/realtek: Enable mute LED on an HP system
  * ALSA: hda/realtek - Enable the headset of ASUS B9450FA with ALC294
  * fix a braino in "sparc32: fix register window handling in genregs32_[gs]et()"
  * i2c: tegra: Fix Maximum transfer size
  * i2c: tegra: Add missing kerneldoc for some fields
  * i2c: tegra: Cleanup kerneldoc comments
  * EDAC/amd64: Add Family 17h Model 30h PCI IDs
  * net: sched: export __netdev_watchdog_up()
  * net: bcmgenet: remove HFB_CTRL access
  * mtd: rawnand: marvell: Fix the condition on a return code
  * fanotify: fix ignore mask logic for events on child and on dir
  * block/bio-integrity: don't free 'buf' if bio_integrity_add_page() failed
  * net: be more gentle about silly gso requests coming from user
  * ANDROID: lib/vdso: do not update timespec if clock_getres() fails
  * Revert "ANDROID: fscrypt: add key removal notifier chain"
  * ANDROID: update the ABI xml and qcom whitelist
  * ANDROID: fs: export vfs_{read|write}
  * f2fs: use kfree() to free variables allocated by match_strdup()
  * f2fs: get the right gc victim section when section has several segments
  * f2fs: fix a race condition between f2fs_write_end_io and f2fs_del_fsync_node_entry
  * f2fs: remove useless truncate in f2fs_collapse_range()
  * f2fs: use kfree() instead of kvfree() to free superblock data
  * f2fs: avoid checkpatch error
  * ANDROID: GKI: update abi definitions now that sdcardfs is gone
  * Revert "ANDROID: sdcardfs: Enable modular sdcardfs"
  * Revert "ANDROID: vfs: Add setattr2 for filesystems with per mount permissions"
  * Revert "ANDROID: vfs: fix export symbol type"
  * Revert "ANDROID: vfs: Add permission2 for filesystems with per mount permissions"
  * Revert "ANDROID: vfs: fix export symbol types"
  * Revert "ANDROID: vfs: add d_canonical_path for stacked filesystem support"
  * Revert "ANDROID: fs: Restore vfs_path_lookup() export"
  * ANDROID: sdcardfs: remove sdcardfs from system
  * Revert "ALSA: usb-audio: Improve frames size computation"
  * Merge 4.19.130 into android-4.19-stable
  * ANDROID: Makefile: append BUILD_NUMBER to version string when defined
  * ANDROID: GKI: Update ABI for incremental fs
  * ANDROID: GKI: Update cuttlefish whitelist
  * ANDROID: GKI: Disable INCREMENTAL_FS on x86 too
  * ANDROID: cpufreq: schedutil: drop cache when update skipped due to rate limit
  * Linux 4.19.130
  * KVM: x86/mmu: Set mmio_value to '0' if reserved #PF can't be generated
  * kvm: x86: Fix reserved bits related calculation errors caused by MKTME
  * kvm: x86: Move kvm_set_mmio_spte_mask() from x86.c to mmu.c
  * md: add feature flag MD_FEATURE_RAID0_LAYOUT
  * Revert "dpaa_eth: fix usage as DSA master, try 3"
  * net: core: device_rename: Use rwsem instead of a seqcount
  * sched/rt, net: Use CONFIG_PREEMPTION.patch
  * kretprobe: Prevent triggering kretprobe from within kprobe_flush_task
  * net: octeon: mgmt: Repair filling of RX ring
  * e1000e: Do not wake up the system via WOL if device wakeup is disabled
  * kprobes: Fix to protect kick_kprobe_optimizer() by kprobe_mutex
  * crypto: algboss - don't wait during notifier callback
  * crypto: algif_skcipher - Cap recv SG list at ctx->used
  * drm/i915/icl+: Fix hotplug interrupt disabling after storm detection
  * drm/i915: Whitelist context-local timestamp in the gen9 cmdparser
  * s390: fix syscall_get_error for compat processes
  * mtd: rawnand: tmio: Fix the probe error path
  * mtd: rawnand: mtk: Fix the probe error path
  * mtd: rawnand: plat_nand: Fix the probe error path
  * mtd: rawnand: socrates: Fix the probe error path
  * mtd: rawnand: oxnas: Fix the probe error path
  * mtd: rawnand: oxnas: Add of_node_put()
  * mtd: rawnand: orion: Fix the probe error path
  * mtd: rawnand: xway: Fix the probe error path
  * mtd: rawnand: sharpsl: Fix the probe error path
  * mtd: rawnand: diskonchip: Fix the probe error path
  * mtd: rawnand: Pass a nand_chip object to nand_release()
  * mtd: rawnand: Pass a nand_chip object to nand_scan()
  * block: nr_sects_write(): Disable preemption on seqcount write
  * x86/boot/compressed: Relax sed symbol type regex for LLVM ld.lld
  * drm/dp_mst: Increase ACT retry timeout to 3s
  * ext4: avoid race conditions when remounting with options that change dax
  * ext4: fix partial cluster initialization when splitting extent
  * selinux: fix double free
  * drm/amdgpu: Replace invalid device ID with a valid device ID
  * drm/qxl: Use correct notify port address when creating cursor ring
  * drm/dp_mst: Reformat drm_dp_check_act_status() a bit
  * drm: encoder_slave: fix refcouting error for modules
  * libata: Use per port sync for detach
  * arm64: hw_breakpoint: Don't invoke overflow handler on uaccess watchpoints
  * block: Fix use-after-free in blkdev_get()
  * afs: afs_write_end() should change i_size under the right lock
  * afs: Fix non-setting of mtime when writing into mmap
  * bcache: fix potential deadlock problem in btree_gc_coalesce
  * ext4: stop overwrite the errcode in ext4_setup_super
  * perf report: Fix NULL pointer dereference in hists__fprintf_nr_sample_events()
  * usb/ehci-platform: Set PM runtime as active on resume
  * usb: host: ehci-platform: add a quirk to avoid stuck
  * usb/xhci-plat: Set PM runtime as active on resume
  * xdp: Fix xsk_generic_xmit errno
  * net/filter: Permit reading NET in load_bytes_relative when MAC not set
  * x86/idt: Keep spurious entries unset in system_vectors
  * scsi: acornscsi: Fix an error handling path in acornscsi_probe()
  * drm/sun4i: hdmi ddc clk: Fix size of m divider
  * ASoC: rt5645: Add platform-data for Asus T101HA
  * ASoC: Intel: bytcr_rt5640: Add quirk for Toshiba Encore WT10-A tablet
  * ASoC: core: only convert non DPCM link to DPCM link
  * afs: Fix memory leak in afs_put_sysnames()
  * selftests/net: in timestamping, strncpy needs to preserve null byte
  * drivers/perf: hisi: Fix wrong value for all counters enable
  * NTB: ntb_test: Fix bug when counting remote files
  * NTB: perf: Fix race condition when run with ntb_test
  * NTB: perf: Fix support for hardware that doesn't have port numbers
  * NTB: perf: Don't require one more memory window than number of peers
  * NTB: Revert the change to use the NTB device dev for DMA allocations
  * NTB: ntb_tool: reading the link file should not end in a NULL byte
  * ntb_tool: pass correct struct device to dma_alloc_coherent
  * ntb_perf: pass correct struct device to dma_alloc_coherent
  * gfs2: fix use-after-free on transaction ail lists
  * blktrace: fix endianness for blk_log_remap()
  * blktrace: fix endianness in get_pdu_int()
  * blktrace: use errno instead of bi_status
  * selftests/vm/pkeys: fix alloc_random_pkey() to make it really random
  * elfnote: mark all .note sections SHF_ALLOC
  * include/linux/bitops.h: avoid clang shift-count-overflow warnings
  * lib/zlib: remove outdated and incorrect pre-increment optimization
  * geneve: change from tx_error to tx_dropped on missing metadata
  * crypto: omap-sham - add proper load balancing support for multicore
  * pinctrl: freescale: imx: Fix an error handling path in 'imx_pinctrl_probe()'
  * pinctrl: imxl: Fix an error handling path in 'imx1_pinctrl_core_probe()'
  * scsi: ufs: Don't update urgent bkops level when toggling auto bkops
  * scsi: iscsi: Fix reference count leak in iscsi_boot_create_kobj
  * gfs2: Allow lock_nolock mount to specify jid=X
  * openrisc: Fix issue with argument clobbering for clone/fork
  * rxrpc: Adjust /proc/net/rxrpc/calls to display call->debug_id not user_ID
  * vfio/mdev: Fix reference count leak in add_mdev_supported_type
  * ASoC: fsl_asrc_dma: Fix dma_chan leak when config DMA channel failed
  * extcon: adc-jack: Fix an error handling path in 'adc_jack_probe()'
  * powerpc/4xx: Don't unmap NULL mbase
  * of: Fix a refcounting bug in __of_attach_node_sysfs()
  * NFSv4.1 fix rpc_call_done assignment for BIND_CONN_TO_SESSION
  * net: sunrpc: Fix off-by-one issues in 'rpc_ntop6'
  * clk: sprd: return correct type of value for _sprd_pll_recalc_rate
  * KVM: PPC: Book3S HV: Ignore kmemleak false positives
  * scsi: ufs-qcom: Fix scheduling while atomic issue
  * clk: bcm2835: Fix return type of bcm2835_register_gate
  * scsi: target: tcmu: Fix a use after free in tcmu_check_expired_queue_cmd()
  * ASoC: fix incomplete error-handling in img_i2s_in_probe.
  * x86/apic: Make TSC deadline timer detection message visible
  * RDMA/iw_cxgb4: cleanup device debugfs entries on ULD remove
  * usb: gadget: Fix issue with config_ep_by_speed function
  * usb: gadget: fix potential double-free in m66592_probe.
  * usb: gadget: lpc32xx_udc: don't dereference ep pointer before null check
  * USB: gadget: udc: s3c2410_udc: Remove pointless NULL check in s3c2410_udc_nuke
  * usb: dwc2: gadget: move gadget resume after the core is in L0 state
  * watchdog: da9062: No need to ping manually before setting timeout
  * IB/cma: Fix ports memory leak in cma_configfs
  * PCI: dwc: Fix inner MSI IRQ domain registration
  * PCI/PTM: Inherit Switch Downstream Port PTM settings from Upstream Port
  * dm zoned: return NULL if dmz_get_zone_for_reclaim() fails to find a zone
  * powerpc/64s/pgtable: fix an undefined behaviour
  * arm64: tegra: Fix ethernet phy-mode for Jetson Xavier
  * scsi: target: tcmu: Userspace must not complete queued commands
  * clk: samsung: exynos5433: Add IGNORE_UNUSED flag to sclk_i2s1
  * fpga: dfl: afu: Corrected error handling levels
  * tty: n_gsm: Fix bogus i++ in gsm_data_kick
  * USB: host: ehci-mxc: Add error handling in ehci_mxc_drv_probe()
  * ASoC: Intel: bytcr_rt5640: Add quirk for Toshiba Encore WT8-A tablet
  * drm/msm/mdp5: Fix mdp5_init error path for failed mdp5_kms allocation
  * usb/ohci-platform: Fix a warning when hibernating
  * vfio-pci: Mask cap zero
  * powerpc/ps3: Fix kexec shutdown hang
  * powerpc/pseries/ras: Fix FWNMI_VALID off by one
  * ipmi: use vzalloc instead of kmalloc for user creation
  * HID: Add quirks for Trust Panora Graphic Tablet
  * tty: n_gsm: Fix waking up upper tty layer when room available
  * tty: n_gsm: Fix SOF skipping
  * powerpc/64: Don't initialise init_task->thread.regs
  * PCI: Fix pci_register_host_bridge() device_register() error handling
  * clk: ti: composite: fix memory leak
  * dlm: remove BUG() before panic()
  * pinctrl: rockchip: fix memleak in rockchip_dt_node_to_map
  * scsi: mpt3sas: Fix double free warnings
  * power: supply: smb347-charger: IRQSTAT_D is volatile
  * power: supply: lp8788: Fix an error handling path in 'lp8788_charger_probe()'
  * scsi: qla2xxx: Fix warning after FC target reset
  * PCI/ASPM: Allow ASPM on links to PCIe-to-PCI/PCI-X Bridges
  * PCI: rcar: Fix incorrect programming of OB windows
  * drivers: base: Fix NULL pointer exception in __platform_driver_probe() if a driver developer is foolish
  * serial: amba-pl011: Make sure we initialize the port.lock spinlock
  * i2c: pxa: fix i2c_pxa_scream_blue_murder() debug output
  * PCI: v3-semi: Fix a memory leak in v3_pci_probe() error handling paths
  * staging: sm750fb: add missing case while setting FB_VISUAL
  * usb: dwc3: gadget: Properly handle failed kick_transfer
  * thermal/drivers/ti-soc-thermal: Avoid dereferencing ERR_PTR
  * slimbus: ngd: get drvdata from correct device
  * tty: hvc: Fix data abort due to race in hvc_open
  * s390/qdio: put thinint indicator after early error
  * ALSA: usb-audio: Fix racy list management in output queue
  * ALSA: usb-audio: Improve frames size computation
  * staging: gasket: Fix mapping refcnt leak when register/store fails
  * staging: gasket: Fix mapping refcnt leak when put attribute fails
  * firmware: qcom_scm: fix bogous abuse of dma-direct internals
  * pinctrl: rza1: Fix wrong array assignment of rza1l_swio_entries
  * scsi: qedf: Fix crash when MFW calls for protocol stats while function is still probing
  * gpio: dwapb: Append MODULE_ALIAS for platform driver
  * ARM: dts: sun8i-h2-plus-bananapi-m2-zero: Fix led polarity
  * scsi: qedi: Do not flush offload work if ARP not resolved
  * arm64: dts: mt8173: fix unit name warnings
  * staging: greybus: fix a missing-check bug in gb_lights_light_config()
  * x86/purgatory: Disable various profiling and sanitizing options
  * apparmor: fix nnp subset test for unconfined
  * scsi: ibmvscsi: Don't send host info in adapter info MAD after LPM
  * scsi: sr: Fix sr_probe() missing deallocate of device minor
  * ASoC: meson: add missing free_irq() in error path
  * apparmor: check/put label on apparmor_sk_clone_security()
  * apparmor: fix introspection of of task mode for unconfined tasks
  * mksysmap: Fix the mismatch of '.L' symbols in System.map
  * NTB: Fix the default port and peer numbers for legacy drivers
  * NTB: ntb_pingpong: Choose doorbells based on port number
  * yam: fix possible memory leak in yam_init_driver
  * pwm: img: Call pm_runtime_put() in pm_runtime_get_sync() failed case
  * powerpc/crashkernel: Take "mem=" option into account
  * PCI: vmd: Filter resource type bits from shadow register
  * nfsd: Fix svc_xprt refcnt leak when setup callback client failed
  * powerpc/perf/hv-24x7: Fix inconsistent output values incase multiple hv-24x7 events run
  * clk: clk-flexgen: fix clock-critical handling
  * scsi: lpfc: Fix lpfc_nodelist leak when processing unsolicited event
  * mfd: wm8994: Fix driver operation if loaded as modules
  * gpio: dwapb: Call acpi_gpiochip_free_interrupts() on GPIO chip de-registration
  * m68k/PCI: Fix a memory leak in an error handling path
  * RDMA/mlx5: Add init2init as a modify command
  * vfio/pci: fix memory leaks in alloc_perm_bits()
  * ps3disk: use the default segment boundary
  * PCI: aardvark: Don't blindly enable ASPM L0s and don't write to read-only register
  * dm mpath: switch paths in dm_blk_ioctl() code path
  * serial: 8250: Fix max baud limit in generic 8250 port
  * usblp: poison URBs upon disconnect
  * clk: samsung: Mark top ISP and CAM clocks on Exynos542x as critical
  * i2c: pxa: clear all master action bits in i2c_pxa_stop_message()
  * f2fs: report delalloc reserve as non-free in statfs for project quota
  * iio: bmp280: fix compensation of humidity
  * scsi: qla2xxx: Fix issue with adapter's stopping state
  * PCI: Allow pci_resize_resource() for devices on root bus
  * ALSA: isa/wavefront: prevent out of bounds write in ioctl
  * ALSA: hda/realtek - Introduce polarity for micmute LED GPIO
  * scsi: qedi: Check for buffer overflow in qedi_set_path()
  * ARM: integrator: Add some Kconfig selections
  * ASoC: davinci-mcasp: Fix dma_chan refcnt leak when getting dma type
  * backlight: lp855x: Ensure regulators are disabled on probe failure
  * clk: qcom: msm8916: Fix the address location of pll->config_reg
  * remoteproc: Fix IDR initialisation in rproc_alloc()
  * iio: pressure: bmp280: Tolerate IRQ before registering
  * i2c: piix4: Detect secondary SMBus controller on AMD AM4 chipsets
  * ASoC: tegra: tegra_wm8903: Support nvidia, headset property
  * clk: sunxi: Fix incorrect usage of round_down()
  * power: supply: bq24257_charger: Replace depends on REGMAP_I2C with select
  * ANDROID: ext4: Optimize match for casefolded encrypted dirs
  * ANDROID: ext4: Handle casefolding with encryption
  * Merge remote-tracking branch 'aosp/upstream-f2fs-stable-linux-4.19.y' into android-4.19-stable
  * xfs: drop I_DIRTY_TIME_EXPIRED
  * ANDROID: extcon: Remove redundant EXPORT_SYMBOL_GPL
  * ANDROID: update the ABI xml representation
  * ANDROID: GKI: cfg80211: add ABI changes for CONFIG_NL80211_TESTMODE
  * ANDROID: gki_defconfig: x86: Enable KERNEL_LZ4
  * ANDROID: GKI: scripts: Makefile: update the lz4 command
  * FROMLIST: f2fs: fix use-after-free when accessing bio->bi_crypt_context
  * Merge remote-tracking branch 'aosp/upstream-f2fs-stable-linux-4.19.y' into android-4.19-stable
  * UPSTREAM: fdt: Update CRC check for rng-seed
  * ANDROID: GKI: Update ABI for incremental fs
  * ANDROID: GKI: Update whitelist and defconfig for incfs
  * ANDROID: Use depmod from the hermetic toolchain
  * Merge 4.19.129 into android-4.19-stable
  * Linux 4.19.129
  * perf symbols: Fix debuginfo search for Ubuntu
  * perf probe: Check address correctness by map instead of _etext
  * perf probe: Fix to check blacklist address correctly
  * perf probe: Do not show the skipped events
  * w1: omap-hdq: cleanup to add missing newline for some dev_dbg
  * mtd: rawnand: pasemi: Fix the probe error path
  * mtd: rawnand: brcmnand: fix hamming oob layout
  * sunrpc: clean up properly in gss_mech_unregister()
  * sunrpc: svcauth_gss_register_pseudoflavor must reject duplicate registrations.
  * kbuild: force to build vmlinux if CONFIG_MODVERSION=y
  * powerpc/64s: Save FSCR to init_task.thread.fscr after feature init
  * powerpc/64s: Don't let DT CPU features set FSCR_DSCR
  * drivers/macintosh: Fix memleak in windfarm_pm112 driver
  * ARM: dts: s5pv210: Set keep-power-in-suspend for SDHCI1 on Aries
  * ARM: dts: at91: sama5d2_ptc_ek: fix vbus pin
  * ARM: dts: exynos: Fix GPIO polarity for thr GalaxyS3 CM36651 sensor's bus
  * ARM: tegra: Correct PL310 Auxiliary Control Register initialization
  * kernel/cpu_pm: Fix uninitted local in cpu_pm
  * alpha: fix memory barriers so that they conform to the specification
  * dm crypt: avoid truncating the logical block size
  * sparc64: fix misuses of access_process_vm() in genregs32_[sg]et()
  * sparc32: fix register window handling in genregs32_[gs]et()
  * gnss: sirf: fix error return code in sirf_probe()
  * pinctrl: samsung: Save/restore eint_mask over suspend for EINT_TYPE GPIOs
  * pinctrl: samsung: Correct setting of eint wakeup mask on s5pv210
  * power: vexpress: add suppress_bind_attrs to true
  * igb: Report speed and duplex as unknown when device is runtime suspended
  * media: ov5640: fix use of destroyed mutex
  * b43_legacy: Fix connection problem with WPA3
  * b43: Fix connection problem with WPA3
  * b43legacy: Fix case where channel status is corrupted
  * Bluetooth: hci_bcm: fix freeing not-requested IRQ
  * media: go7007: fix a miss of snd_card_free
  * carl9170: remove P2P_GO support
  * e1000e: Relax condition to trigger reset for ME workaround
  * e1000e: Disable TSO for buffer overrun workaround
  * PCI: Program MPS for RCiEP devices
  * ima: Call ima_calc_boot_aggregate() in ima_eventdigest_init()
  * btrfs: fix wrong file range cleanup after an error filling dealloc range
  * btrfs: fix error handling when submitting direct I/O bio
  * PCI: Generalize multi-function power dependency device links
  * PCI: Unify ACS quirk desired vs provided checking
  * PCI: Make ACS quirk implementations more uniform
  * serial: 8250_pci: Move Pericom IDs to pci_ids.h
  * PCI: Add Loongson vendor ID
  * x86/amd_nb: Add Family 19h PCI IDs
  * PCI: vmd: Add device id for VMD device 8086:9A0B
  * PCI: Add Amazon's Annapurna Labs vendor ID
  * PCI: Add Genesys Logic, Inc. Vendor ID
  * ALSA: lx6464es - add support for LX6464ESe pci express variant
  * x86/amd_nb: Add PCI device IDs for family 17h, model 70h
  * PCI: mediatek: Add controller support for MT7629
  * PCI: Enable NVIDIA HDA controllers
  * PCI: Add NVIDIA GPU multi-function power dependencies
  * PCI: Add Synopsys endpoint EDDA Device ID
  * misc: pci_endpoint_test: Add support to test PCI EP in AM654x
  * misc: pci_endpoint_test: Add the layerscape EP device support
  * PCI: Move Rohm Vendor ID to generic list
  * PCI: Move Synopsys HAPS platform device IDs
  * PCI: add USR vendor id and use it in r8169 and w6692 driver
  * x86/amd_nb: Add PCI device IDs for family 17h, model 30h
  * hwmon/k10temp, x86/amd_nb: Consolidate shared device IDs
  * pci:ipmi: Move IPMI PCI class id defines to pci_ids.h
  * PCI: Remove unused NFP32xx IDs
  * PCI: Add ACS quirk for Intel Root Complex Integrated Endpoints
  * PCI: Add ACS quirk for iProc PAXB
  * PCI: Avoid FLR for AMD Starship USB 3.0
  * PCI: Avoid FLR for AMD Matisse HD Audio & USB 3.0
  * PCI: Avoid Pericom USB controller OHCI/EHCI PME# defect
  * ext4: fix race between ext4_sync_parent() and rename()
  * ext4: fix error pointer dereference
  * ext4: fix EXT_MAX_EXTENT/INDEX to check for zeroed eh_max
  * evm: Fix possible memory leak in evm_calc_hmac_or_hash()
  * ima: Directly assign the ima_default_policy pointer to ima_rules
  * ima: Fix ima digest hash table key calculation
  * mm: initialize deferred pages with interrupts enabled
  * mm: thp: make the THP mapcount atomic against __split_huge_pmd_locked()
  * btrfs: send: emit file capabilities after chown
  * btrfs: include non-missing as a qualifier for the latest_bdev
  * string.h: fix incompatibility between FORTIFY_SOURCE and KASAN
  * platform/x86: intel-vbtn: Only blacklist SW_TABLET_MODE on the 9 / "Laptop" chasis-type
  * platform/x86: intel-hid: Add a quirk to support HP Spectre X2 (2015)
  * platform/x86: hp-wmi: Convert simple_strtoul() to kstrtou32()
  * cpuidle: Fix three reference count leaks
  * spi: dw: Return any value retrieved from the dma_transfer callback
  * mmc: sdhci-esdhc-imx: fix the mask for tuning start point
  * ixgbe: fix signed-integer-overflow warning
  * mmc: via-sdmmc: Respect the cmd->busy_timeout from the mmc core
  * staging: greybus: sdio: Respect the cmd->busy_timeout from the mmc core
  * mmc: sdhci-msm: Set SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12 quirk
  * bcache: fix refcount underflow in bcache_device_free()
  * MIPS: Fix IRQ tracing when call handle_fpe() and handle_msa_fpe()
  * PCI: Don't disable decoding when mmio_always_on is set
  * macvlan: Skip loopback packets in RX handler
  * btrfs: qgroup: mark qgroup inconsistent if we're inherting snapshot to a new qgroup
  * m68k: mac: Don't call via_flush_cache() on Mac IIfx
  * x86/mm: Stop printing BRK addresses
  * crypto: stm32/crc32 - fix multi-instance
  * crypto: stm32/crc32 - fix run-time self test issue.
  * crypto: stm32/crc32 - fix ext4 chksum BUG_ON()
  * mips: Add udelay lpj numbers adjustment
  * mips: MAAR: Use more precise address mask
  * x86/boot: Correct relocation destination on old linkers
  * mwifiex: Fix memory corruption in dump_station
  * rtlwifi: Fix a double free in _rtl_usb_tx_urb_setup()
  * net/mlx5e: IPoIB, Drop multicast packets that this interface sent
  * veth: Adjust hard_start offset on redirect XDP frames
  * md: don't flush workqueue unconditionally in md_open
  * mt76: avoid rx reorder buffer overflow
  * net: qed*: Reduce RX and TX default ring count when running inside kdump kernel
  * wcn36xx: Fix error handling path in 'wcn36xx_probe()'
  * ath10k: Remove msdu from idr when management pkt send fails
  * nvme: refine the Qemu Identify CNS quirk
  * platform/x86: intel-vbtn: Also handle tablet-mode switch on "Detachable" and "Portable" chassis-types
  * platform/x86: intel-vbtn: Do not advertise switches to userspace if they are not there
  * platform/x86: intel-vbtn: Split keymap into buttons and switches parts
  * platform/x86: intel-vbtn: Use acpi_evaluate_integer()
  * xfs: fix duplicate verification from xfs_qm_dqflush()
  * xfs: reset buffer write failure state on successful completion
  * kgdb: Fix spurious true from in_dbg_master()
  * mips: cm: Fix an invalid error code of INTVN_*_ERR
  * MIPS: Truncate link address into 32bit for 32bit kernel
  * Crypto/chcr: fix for ccm(aes) failed test
  * xfs: clean up the error handling in xfs_swap_extents
  * powerpc/spufs: fix copy_to_user while atomic
  * net: allwinner: Fix use correct return type for ndo_start_xmit()
  * media: cec: silence shift wrapping warning in __cec_s_log_addrs()
  * net: lpc-enet: fix error return code in lpc_mii_init()
  * drivers/perf: hisi: Fix typo in events attribute array
  * sched/core: Fix illegal RCU from offline CPUs
  * exit: Move preemption fixup up, move blocking operations down
  * lib/mpi: Fix 64-bit MIPS build with Clang
  * net: bcmgenet: set Rx mode before starting netif
  * selftests/bpf: Fix memory leak in extract_build_id()
  * netfilter: nft_nat: return EOPNOTSUPP if type or flags are not supported
  * audit: fix a net reference leak in audit_list_rules_send()
  * Bluetooth: btbcm: Add 2 missing models to subver tables
  * MIPS: Make sparse_init() using top-down allocation
  * media: platform: fcp: Set appropriate DMA parameters
  * media: dvb: return -EREMOTEIO on i2c transfer failure.
  * audit: fix a net reference leak in audit_send_reply()
  * dt-bindings: display: mediatek: control dpi pins mode to avoid leakage
  * e1000: Distribute switch variables for initialization
  * tools api fs: Make xxx__mountpoint() more scalable
  * brcmfmac: fix wrong location to get firmware feature
  * staging: android: ion: use vmap instead of vm_map_ram
  * net: vmxnet3: fix possible buffer overflow caused by bad DMA value in vmxnet3_get_rss()
  * x86/kvm/hyper-v: Explicitly align hcall param for kvm_hyperv_exit
  * spi: dw: Fix Rx-only DMA transfers
  * mmc: meson-mx-sdio: trigger a soft reset after a timeout or CRC error
  * batman-adv: Revert "disable ethtool link speed detection when auto negotiation off"
  * ARM: 8978/1: mm: make act_mm() respect THREAD_SIZE
  * btrfs: do not ignore error from btrfs_next_leaf() when inserting checksums
  * clocksource: dw_apb_timer_of: Fix missing clockevent timers
  * clocksource: dw_apb_timer: Make CPU-affiliation being optional
  * spi: dw: Enable interrupts in accordance with DMA xfer mode
  * kgdb: Prevent infinite recursive entries to the debugger
  * kgdb: Disable WARN_CONSOLE_UNLOCKED for all kgdb
  * Bluetooth: Add SCO fallback for invalid LMP parameters error
  * MIPS: Loongson: Build ATI Radeon GPU driver as module
  * ixgbe: Fix XDP redirect on archs with PAGE_SIZE above 4K
  * arm64: insn: Fix two bugs in encoding 32-bit logical immediates
  * spi: dw: Zero DMA Tx and Rx configurations on stack
  * arm64: cacheflush: Fix KGDB trap detection
  * efi/libstub/x86: Work around LLVM ELF quirk build regression
  * net: ena: fix error returning in ena_com_get_hash_function()
  * net: atlantic: make hw_get_regs optional
  * spi: pxa2xx: Apply CS clk quirk to BXT
  * objtool: Ignore empty alternatives
  * media: si2157: Better check for running tuner in init
  * crypto: ccp -- don't "select" CONFIG_DMADEVICES
  * drm: bridge: adv7511: Extend list of audio sample rates
  * ACPI: GED: use correct trigger type field in _Exx / _Lxx handling
  * KVM: arm64: Synchronize sysreg state on injecting an AArch32 exception
  * xen/pvcalls-back: test for errors when calling backend_connect()
  * mmc: sdio: Fix potential NULL pointer error in mmc_sdio_init_card()
  * ARM: dts: at91: sama5d2_ptc_ek: fix sdmmc0 node description
  * mmc: sdhci-msm: Clear tuning done flag while hs400 tuning
  * agp/intel: Reinforce the barrier after GTT updates
  * perf: Add cond_resched() to task_function_call()
  * fat: don't allow to mount if the FAT length == 0
  * mm/slub: fix a memory leak in sysfs_slab_add()
  * drm/vkms: Hold gem object while still in-use
  * Smack: slab-out-of-bounds in vsscanf
  * ath9k: Fix general protection fault in ath9k_hif_usb_rx_cb
  * ath9x: Fix stack-out-of-bounds Write in ath9k_hif_usb_rx_cb
  * ath9k: Fix use-after-free Write in ath9k_htc_rx_msg
  * ath9k: Fix use-after-free Read in ath9k_wmi_ctrl_rx
  * scsi: megaraid_sas: TM command refire leads to controller firmware crash
  * KVM: arm64: Make vcpu_cp1x() work on Big Endian hosts
  * KVM: MIPS: Fix VPN2_MASK definition for variable cpu_vmbits
  * KVM: MIPS: Define KVM_ENTRYHI_ASID to cpu_asid_mask(&boot_cpu_data)
  * KVM: nVMX: Consult only the "basic" exit reason when routing nested exit
  * KVM: nSVM: leave ASID aside in copy_vmcb_control_area
  * KVM: nSVM: fix condition for filtering async PF
  * video: fbdev: w100fb: Fix a potential double free.
  * proc: Use new_inode not new_inode_pseudo
  * ovl: initialize error in ovl_copy_xattr
  * selftests/net: in rxtimestamp getopt_long needs terminating null entry
  * crypto: virtio: Fix dest length calculation in __virtio_crypto_skcipher_do_req()
  * crypto: virtio: Fix src/dst scatterlist calculation in __virtio_crypto_skcipher_do_req()
  * crypto: virtio: Fix use-after-free in virtio_crypto_skcipher_finalize_req()
  * spi: pxa2xx: Fix runtime PM ref imbalance on probe error
  * spi: pxa2xx: Balance runtime PM enable/disable on error
  * spi: bcm2835: Fix controller unregister order
  * spi: pxa2xx: Fix controller unregister order
  * spi: Fix controller unregister order
  * spi: No need to assign dummy value in spi_unregister_controller()
  * x86/speculation: PR_SPEC_FORCE_DISABLE enforcement for indirect branches.
  * x86/speculation: Avoid force-disabling IBPB based on STIBP and enhanced IBRS.
  * x86/speculation: Add support for STIBP always-on preferred mode
  * x86/speculation: Change misspelled STIPB to STIBP
  * KVM: x86: only do L1TF workaround on affected processors
  * KVM: x86/mmu: Consolidate "is MMIO SPTE" code
  * kvm: x86: Fix L1TF mitigation for shadow MMU
  * KVM: x86: Fix APIC page invalidation race
  * x86/{mce,mm}: Unmap the entire page if the whole page is affected and poisoned
  * ALSA: pcm: disallow linking stream to itself
  * crypto: cavium/nitrox - Fix 'nitrox_get_first_device()' when ndevlist is fully iterated
  * PM: runtime: clk: Fix clk_pm_runtime_get() error path
  * spi: bcm-qspi: when tx/rx buffer is NULL set to 0
  * spi: bcm2835aux: Fix controller unregister order
  * spi: dw: Fix controller unregister order
  * nilfs2: fix null pointer dereference at nilfs_segctor_do_construct()
  * cgroup, blkcg: Prepare some symbols for module and !CONFIG_CGROUP usages
  * ACPI: PM: Avoid using power resources if there are none for D0
  * ACPI: GED: add support for _Exx / _Lxx handler methods
  * ACPI: CPPC: Fix reference count leak in acpi_cppc_processor_probe()
  * ACPI: sysfs: Fix reference count leak in acpi_sysfs_add_hotplug_profile()
  * ALSA: usb-audio: Add vendor, product and profile name for HP Thunderbolt Dock
  * ALSA: usb-audio: Fix inconsistent card PM state after resume
  * ALSA: hda/realtek - add a pintbl quirk for several Lenovo machines
  * ALSA: es1688: Add the missed snd_card_free()
  * efi/efivars: Add missing kobject_put() in sysfs entry creation error path
  * x86/reboot/quirks: Add MacBook6,1 reboot quirk
  * x86/speculation: Prevent rogue cross-process SSBD shutdown
  * x86/PCI: Mark Intel C620 MROMs as having non-compliant BARs
  * x86_64: Fix jiffies ODR violation
  * btrfs: tree-checker: Check level for leaves and nodes
  * aio: fix async fsync creds
  * mm: add kvfree_sensitive() for freeing sensitive data objects
  * perf probe: Accept the instance number of kretprobe event
  * x86/cpu/amd: Make erratum #1054 a legacy erratum
  * RDMA/uverbs: Make the event_queue fds return POLLERR when disassociated
  * ath9k_htc: Silence undersized packet warnings
  * powerpc/xive: Clear the page tables for the ESB IO mapping
  * drivers/net/ibmvnic: Update VNIC protocol version reporting
  * Input: synaptics - add a second working PNP_ID for Lenovo T470s
  * sched/fair: Don't NUMA balance for kthreads
  * ARM: 8977/1: ptrace: Fix mask for thumb breakpoint hook
  * Input: mms114 - fix handling of mms345l
  * crypto: talitos - fix ECB and CBC algs ivsize
  * btrfs: Detect unbalanced tree with empty leaf before crashing btree operations
  * btrfs: merge btrfs_find_device and find_device
  * lib: Reduce user_access_begin() boundaries in strncpy_from_user() and strnlen_user()
  * x86: uaccess: Inhibit speculation past access_ok() in user_access_begin()
  * arch/openrisc: Fix issues with access_ok()
  * Fix 'acccess_ok()' on alpha and SH
  * make 'user_access_begin()' do 'access_ok()'
  * selftests: bpf: fix use of undeclared RET_IF macro
  * tun: correct header offsets in napi frags mode
  * vxlan: Avoid infinite loop when suppressing NS messages with invalid options
  * bridge: Avoid infinite loop when suppressing NS messages with invalid options
  * net_failover: fixed rollback in net_failover_open()
  * ipv6: fix IPV6_ADDRFORM operation logic
  * writeback: Drop I_DIRTY_TIME_EXPIRE
  * writeback: Fix sync livelock due to b_dirty_time processing
  * writeback: Avoid skipping inode writeback
  * writeback: Protect inode->i_io_list with inode->i_lock
  * Revert "writeback: Avoid skipping inode writeback"
  * ANDROID: gki_defconfig: increase vbus_draw to 500mA
  * fscrypt: remove stale definition
  * fs-verity: remove unnecessary extern keywords
  * fs-verity: fix all kerneldoc warnings
  * fscrypt: add support for IV_INO_LBLK_32 policies
  * fscrypt: make test_dummy_encryption use v2 by default
  * fscrypt: support test_dummy_encryption=v2
  * fscrypt: add fscrypt_add_test_dummy_key()
  * linux/parser.h: add include guards
  * fscrypt: remove unnecessary extern keywords
  * fscrypt: name all function parameters
  * fscrypt: fix all kerneldoc warnings
  * ANDROID: Update the ABI
  * ANDROID: GKI: power: power-supply: Add POWER_SUPPLY_PROP_CHARGER_STATUS property
  * ANDROID: GKI: add dev to usb_gsi_request
  * ANDROID: GKI: dma-buf: add dent_count to dma_buf
  * Merge remote-tracking branch 'aosp/upstream-f2fs-stable-linux-4.19.y' into android-4.19-stable
  * ANDROID: Update the ABI xml and whitelist
  * ANDROID: GKI: update whitelist
  * ANDROID: extcon: Export symbol of `extcon_get_edev_name`
  * ANDROID: kbuild: merge more sections with LTO
  * UPSTREAM: timekeeping/vsyscall: Update VDSO data unconditionally
  * ANDROID: GKI: Revert "genetlink: disallow subscribing to unknown mcast groups"
  * BACKPORT: usb: musb: Add support for MediaTek musb controller
  * UPSTREAM: usb: musb: Add musb_clearb/w() interface
  * UPSTREAM: usb: musb: Add noirq type of dma create interface
  * UPSTREAM: usb: musb: Add get/set toggle hooks
  * UPSTREAM: dt-bindings: usb: musb: Add support for MediaTek musb controller
  * Merge 4.19.128 into android-4.19-stable
  * FROMGIT: driver core: Remove unnecessary is_fwnode_dev variable in device_add()
  * FROMGIT: driver core: Remove check in driver_deferred_probe_force_trigger()
  * FROMGIT: of: platform: Batch fwnode parsing when adding all top level devices
  * FROMGIT: BACKPORT: driver core: fw_devlink: Add support for batching fwnode parsing
  * BACKPORT: driver core: Look for waiting consumers only for a fwnode's primary device
  * BACKPORT: driver core: Add device links from fwnode only for the primary device
  * Linux 4.19.128
  * Revert "net/mlx5: Annotate mutex destroy for root ns"
  * uprobes: ensure that uprobe->offset and ->ref_ctr_offset are properly aligned
  * x86/speculation: Add Ivy Bridge to affected list
  * x86/speculation: Add SRBDS vulnerability and mitigation documentation
  * x86/speculation: Add Special Register Buffer Data Sampling (SRBDS) mitigation
  * x86/cpu: Add 'table' argument to cpu_matches()
  * x86/cpu: Add a steppings field to struct x86_cpu_id
  * nvmem: qfprom: remove incorrect write support
  * CDC-ACM: heed quirk also in error handling
  * staging: rtl8712: Fix IEEE80211_ADDBA_PARAM_BUF_SIZE_MASK
  * tty: hvc_console, fix crashes on parallel open/close
  * vt: keyboard: avoid signed integer overflow in k_ascii
  * usb: musb: Fix runtime PM imbalance on error
  * usb: musb: start session in resume for host port
  * iio: vcnl4000: Fix i2c swapped word reading.
  * USB: serial: option: add Telit LE910C1-EUX compositions
  * USB: serial: usb_wwan: do not resubmit rx urb on fatal errors
  * USB: serial: qcserial: add DW5816e QDL support
  * net: check untrusted gso_size at kernel entry
  * vsock: fix timeout in vsock_accept()
  * NFC: st21nfca: add missed kfree_skb() in an error path
  * net: usb: qmi_wwan: add Telit LE910C1-EUX composition
  * l2tp: do not use inet_hash()/inet_unhash()
  * l2tp: add sk_family checks to l2tp_validate_socket
  * devinet: fix memleak in inetdev_init()
  * Revert "ANDROID: Remove default y on BRIDGE_IGMP_SNOOPING"
  * ANDROID: Update the ABI xml and whitelist
  * ANDROID: GKI: update whitelist
  * ANDROID: arch: arm64: vdso: export the symbols for time()
  * ANDROID: Incremental fs: Remove dependency on PKCS7_MESSAGE_PARSER
  * ANDROID: dm-bow: Add block_size option
  * f2fs: attach IO flags to the missing cases
  * f2fs: add node_io_flag for bio flags likewise data_io_flag
  * f2fs: remove unused parameter of f2fs_put_rpages_mapping()
  * f2fs: handle readonly filesystem in f2fs_ioc_shutdown()
  * f2fs: avoid utf8_strncasecmp() with unstable name
  * f2fs: don't return vmalloc() memory from f2fs_kmalloc()
  * ANDROID: GKI: set CONFIG_BLK_DEV_LOOP_MIN_COUNT to 16
  * ANDROID: Incremental fs: Cache successful hash calculations
  * ANDROID: Incremental fs: Fix four error-path bugs
  * Merge 4.19.127 into android-4.19-stable
  * Linux 4.19.127
  * net: smsc911x: Fix runtime PM imbalance on error
  * net: ethernet: stmmac: Enable interface clocks on probe for IPQ806x
  * net/ethernet/freescale: rework quiesce/activate for ucc_geth
  * null_blk: return error for invalid zone size
  * s390/mm: fix set_huge_pte_at() for empty ptes
  * drm/edid: Add Oculus Rift S to non-desktop list
  * net: bmac: Fix read of MAC address from ROM
  * x86/mmiotrace: Use cpumask_available() for cpumask_var_t variables
  * i2c: altera: Fix race between xfer_msg and isr thread
  * evm: Fix RCU list related warnings
  * ARC: [plat-eznps]: Restrict to CONFIG_ISA_ARCOMPACT
  * ARC: Fix ICCM & DCCM runtime size checks
  * s390/ftrace: save traced function caller
  * spi: dw: use "smp_mb()" to avoid sending spi data error
  * powerpc/powernv: Avoid re-registration of imc debugfs directory
  * scsi: hisi_sas: Check sas_port before using it
  * drm/i915: fix port checks for MST support on gen >= 11
  * airo: Fix read overflows sending packets
  * net: dsa: mt7530: set CPU port to fallback mode
  * scsi: ufs: Release clock if DMA map fails
  * mmc: fix compilation of user API
  * kernel/relay.c: handle alloc_percpu returning NULL in relay_open
  * p54usb: add AirVasT USB stick device-id
  * HID: i2c-hid: add Schneider SCL142ALM to descriptor override
  * HID: sony: Fix for broken buttons on DS3 USB dongles
  * mm: Fix mremap not considering huge pmd devmap
  * libnvdimm: Fix endian conversion issues 
  * Revert "cgroup: Add memory barriers to plug cgroup_rstat_updated() race window"
  * f2fs: fix retry logic in f2fs_write_cache_pages()
  * ANDROID: Update ABI representation
  * Merge 4.19.126 into android-4.19-stable
  * Linux 4.19.126
  * mm/vmalloc.c: don't dereference possible NULL pointer in __vunmap()
  * netfilter: nf_conntrack_pptp: fix compilation warning with W=1 build
  * bonding: Fix reference count leak in bond_sysfs_slave_add.
  * crypto: chelsio/chtls: properly set tp->lsndtime
  * qlcnic: fix missing release in qlcnic_83xx_interrupt_test.
  * xsk: Add overflow check for u64 division, stored into u32
  * bnxt_en: Fix accumulation of bp->net_stats_prev.
  * esp6: get the right proto for transport mode in esp6_gso_encap
  * netfilter: nf_conntrack_pptp: prevent buffer overflows in debug code
  * netfilter: nfnetlink_cthelper: unbreak userspace helper support
  * netfilter: ipset: Fix subcounter update skip
  * netfilter: nft_reject_bridge: enable reject with bridge vlan
  * ip_vti: receive ipip packet by calling ip_tunnel_rcv
  * vti4: eliminated some duplicate code.
  * xfrm: fix error in comment
  * xfrm: fix a NULL-ptr deref in xfrm_local_error
  * xfrm: fix a warning in xfrm_policy_insert_list
  * xfrm interface: fix oops when deleting a x-netns interface
  * xfrm: call xfrm_output_gso when inner_protocol is set in xfrm_output
  * xfrm: allow to accept packets with ipv6 NEXTHDR_HOP in xfrm_input
  * copy_xstate_to_kernel(): don't leave parts of destination uninitialized
  * x86/dma: Fix max PFN arithmetic overflow on 32 bit systems
  * mac80211: mesh: fix discovery timer re-arming issue / crash
  * RDMA/core: Fix double destruction of uobject
  * mmc: core: Fix recursive locking issue in CQE recovery path
  * parisc: Fix kernel panic in mem_init()
  * iommu: Fix reference count leak in iommu_group_alloc.
  * include/asm-generic/topology.h: guard cpumask_of_node() macro argument
  * fs/binfmt_elf.c: allocate initialized memory in fill_thread_core_info()
  * mm: remove VM_BUG_ON(PageSlab()) from page_mapcount()
  * IB/ipoib: Fix double free of skb in case of multicast traffic in CM mode
  * libceph: ignore pool overlay and cache logic on redirects
  * ALSA: hda/realtek - Add new codec supported for ALC287
  * ALSA: usb-audio: Quirks for Gigabyte TRX40 Aorus Master onboard audio
  * exec: Always set cap_ambient in cap_bprm_set_creds
  * ALSA: usb-audio: mixer: volume quirk for ESS Technology Asus USB DAC
  * ALSA: hda/realtek - Add a model for Thinkpad T570 without DAC workaround
  * ALSA: hwdep: fix a left shifting 1 by 31 UB bug
  * RDMA/pvrdma: Fix missing pci disable in pvrdma_pci_probe()
  * mmc: block: Fix use-after-free issue for rpmb
  * ARM: dts: bcm: HR2: Fix PPI interrupt types
  * ARM: dts: bcm2835-rpi-zero-w: Fix led polarity
  * ARM: dts/imx6q-bx50v3: Set display interface clock parents
  * IB/qib: Call kobject_put() when kobject_init_and_add() fails
  * gpio: exar: Fix bad handling for ida_simple_get error path
  * ARM: uaccess: fix DACR mismatch with nested exceptions
  * ARM: uaccess: integrate uaccess_save and uaccess_restore
  * ARM: uaccess: consolidate uaccess asm to asm/uaccess-asm.h
  * ARM: 8843/1: use unified assembler in headers
  * ARM: 8970/1: decompressor: increase tag size
  * Input: synaptics-rmi4 - fix error return code in rmi_driver_probe()
  * Input: synaptics-rmi4 - really fix attn_data use-after-free
  * Input: i8042 - add ThinkPad S230u to i8042 reset list
  * Input: dlink-dir685-touchkeys - fix a typo in driver name
  * Input: xpad - add custom init packet for Xbox One S controllers
  * Input: evdev - call input_flush_device() on release(), not flush()
  * Input: usbtouchscreen - add support for BonXeon TP
  * samples: bpf: Fix build error
  * cifs: Fix null pointer check in cifs_read
  * riscv: stacktrace: Fix undefined reference to `walk_stackframe'
  * IB/i40iw: Remove bogus call to netdev_master_upper_dev_get()
  * net: freescale: select CONFIG_FIXED_PHY where needed
  * usb: gadget: legacy: fix redundant initialization warnings
  * usb: dwc3: pci: Enable extcon driver for Intel Merrifield
  * cachefiles: Fix race between read_waiter and read_copier involving op->to_do
  * gfs2: move privileged user check to gfs2_quota_lock_check
  * net: microchip: encx24j600: add missed kthread_stop
  * ALSA: usb-audio: add mapping for ASRock TRX40 Creator
  * gpio: tegra: mask GPIO IRQs during IRQ shutdown
  * ARM: dts: rockchip: fix pinctrl sub nodename for spi in rk322x.dtsi
  * ARM: dts: rockchip: swap clock-names of gpu nodes
  * arm64: dts: rockchip: swap interrupts interrupt-names rk3399 gpu node
  * arm64: dts: rockchip: fix status for &gmac2phy in rk3328-evb.dts
  * ARM: dts: rockchip: fix phy nodename for rk3228-evb
  * mlxsw: spectrum: Fix use-after-free of split/unsplit/type_set in case reload fails
  * net/mlx4_core: fix a memory leak bug.
  * net: sun: fix missing release regions in cas_init_one().
  * net/mlx5: Annotate mutex destroy for root ns
  * net/mlx5e: Update netdev txq on completions during closure
  * sctp: Start shutdown on association restart if in SHUTDOWN-SENT state and socket is closed
  * sctp: Don't add the shutdown timer if its already been added
  * r8152: support additional Microsoft Surface Ethernet Adapter variant
  * net sched: fix reporting the first-time use timestamp
  * net: revert "net: get rid of an signed integer overflow in ip_idents_reserve()"
  * net: qrtr: Fix passing invalid reference to qrtr_local_enqueue()
  * net/mlx5: Add command entry handling completion
  * net: ipip: fix wrong address family in init error path
  * net: inet_csk: Fix so_reuseport bind-address cache in tb->fast*
  * __netif_receive_skb_core: pass skb by reference
  * net: dsa: mt7530: fix roaming from DSA user ports
  * dpaa_eth: fix usage as DSA master, try 3
  * ax25: fix setsockopt(SO_BINDTODEVICE)
  * ANDROID: modules: fix lockprove warning
  * FROMGIT: USB: dummy-hcd: use configurable endpoint naming scheme
  * UPSTREAM: usb: raw-gadget: fix null-ptr-deref when reenabling endpoints
  * UPSTREAM: usb: raw-gadget: documentation updates
  * UPSTREAM: usb: raw-gadget: support stalling/halting/wedging endpoints
  * UPSTREAM: usb: raw-gadget: fix gadget endpoint selection
  * UPSTREAM: usb: raw-gadget: improve uapi headers comments
  * UPSTREAM: usb: raw-gadget: fix return value of ep read ioctls
  * UPSTREAM: usb: raw-gadget: fix raw_event_queue_fetch locking
  * UPSTREAM: usb: raw-gadget: Fix copy_to/from_user() checks
  * f2fs: fix wrong discard space
  * f2fs: compress: don't compress any datas after cp stop
  * f2fs: remove unneeded return value of __insert_discard_tree()
  * f2fs: fix wrong value of tracepoint parameter
  * f2fs: protect new segment allocation in expand_inode_data
  * f2fs: code cleanup by removing ifdef macro surrounding
  * writeback: Avoid skipping inode writeback
  * ANDROID: GKI: Update the ABI
  * ANDROID: GKI: update whitelist
  * ANDROID: GKI: support mm_event for FS/IO/UFS path
  * ANDROID: net: bpf: permit redirect from ingress L3 to egress L2 devices at near max mtu
  * FROMGIT: driver core: Update device link status correctly for SYNC_STATE_ONLY links
  * UPSTREAM: driver core: Fix handling of SYNC_STATE_ONLY + STATELESS device links
  * BACKPORT: driver core: Fix SYNC_STATE_ONLY device link implementation
  * ANDROID: Bulk update the ABI xml and qcom whitelist
  * Revert "ANDROID: Incremental fs: Avoid continually recalculating hashes"
  * Merge 4.19.125 into android-4.19-stable
  * Merge remote-tracking branch 'aosp/upstream-f2fs-stable-linux-4.19.y' into android-4.19-stable
  * Linux 4.19.125
  * rxrpc: Fix ack discard
  * rxrpc: Trace discarded ACKs
  * iio: adc: stm32-dfsdm: fix device used to request dma
  * iio: adc: stm32-dfsdm: Use dma_request_chan() instead dma_request_slave_channel()
  * iio: adc: stm32-adc: fix device used to request dma
  * iio: adc: stm32-adc: Use dma_request_chan() instead dma_request_slave_channel()
  * x86/unwind/orc: Fix unwind_get_return_address_ptr() for inactive tasks
  * rxrpc: Fix a memory leak in rxkad_verify_response()
  * rapidio: fix an error in get_user_pages_fast() error handling
  * ipack: tpci200: fix error return code in tpci200_register()
  * mei: release me_cl object reference
  * misc: rtsx: Add short delay after exit from ASPM
  * iio: dac: vf610: Fix an error handling path in 'vf610_dac_probe()'
  * iio: sca3000: Remove an erroneous 'get_device()'
  * staging: greybus: Fix uninitialized scalar variable
  * staging: iio: ad2s1210: Fix SPI reading
  * Revert "gfs2: Don't demote a glock until its revokes are written"
  * brcmfmac: abort and release host after error
  * tty: serial: qcom_geni_serial: Fix wrap around of TX buffer
  * cxgb4/cxgb4vf: Fix mac_hlist initialization and free
  * cxgb4: free mac_hlist properly
  * net: bcmgenet: abort suspend on error
  * net: bcmgenet: code movement
  * Revert "net/ibmvnic: Fix EOI when running in XIVE mode"
  * media: fdp1: Fix R-Car M3-N naming in debug message
  * thunderbolt: Drop duplicated get_switch_at_route()
  * staging: most: core: replace strcpy() by strscpy()
  * libnvdimm/btt: Fix LBA masking during 'free list' population
  * libnvdimm/btt: Remove unnecessary code in btt_freelist_init
  * nfit: Add Hyper-V NVDIMM DSM command set to white list
  * powerpc/64s: Disable STRICT_KERNEL_RWX
  * powerpc: Remove STRICT_KERNEL_RWX incompatibility with RELOCATABLE
  * drm/i915/gvt: Init DPLL/DDI vreg for virtual display instead of inheritance.
  * dmaengine: owl: Use correct lock in owl_dma_get_pchan()
  * dmaengine: tegra210-adma: Fix an error handling path in 'tegra_adma_probe()'
  * apparmor: Fix aa_label refcnt leak in policy_update
  * apparmor: fix potential label refcnt leak in aa_change_profile
  * apparmor: Fix use-after-free in aa_audit_rule_init
  * drm/etnaviv: fix perfmon domain interation
  * ALSA: hda/realtek - Add more fixup entries for Clevo machines
  * ALSA: hda/realtek - Fix silent output on Gigabyte X570 Aorus Xtreme
  * ALSA: pcm: fix incorrect hw_base increase
  * ALSA: iec1712: Initialize STDSP24 properly when using the model=staudio option
  * padata: purge get_cpu and reorder_via_wq from padata_do_serial
  * padata: initialize pd->cpu with effective cpumask
  * padata: Replace delayed timer with immediate workqueue in padata_reorder
  * ARM: futex: Address build warning
  * platform/x86: asus-nb-wmi: Do not load on Asus T100TA and T200TA
  * USB: core: Fix misleading driver bug report
  * stmmac: fix pointer check after utilization in stmmac_interrupt
  * ceph: fix double unlock in handle_cap_export()
  * HID: quirks: Add HID_QUIRK_NO_INIT_REPORTS quirk for Dell K12A keyboard-dock
  * gtp: set NLM_F_MULTI flag in gtp_genl_dump_pdp()
  * x86/apic: Move TSC deadline timer debug printk
  * HID: i2c-hid: reset Synaptics SYNA2393 on resume
  * scsi: ibmvscsi: Fix WARN_ON during event pool release
  * component: Silence bind error on -EPROBE_DEFER
  * aquantia: Fix the media type of AQC100 ethernet controller in the driver
  * vhost/vsock: fix packet delivery order to monitoring devices
  * configfs: fix config_item refcnt leak in configfs_rmdir()
  * scsi: qla2xxx: Delete all sessions before unregister local nvme port
  * scsi: qla2xxx: Fix hang when issuing nvme disconnect-all in NPIV
  * HID: alps: ALPS_1657 is too specific; use U1_UNICORN_LEGACY instead
  * HID: alps: Add AUI1657 device ID
  * HID: multitouch: add eGalaxTouch P80H84 support
  * gcc-common.h: Update for GCC 10
  * ubi: Fix seq_file usage in detailed_erase_block_info debugfs file
  * i2c: mux: demux-pinctrl: Fix an error handling path in 'i2c_demux_pinctrl_probe()'
  * iommu/amd: Fix over-read of ACPI UID from IVRS table
  * ubifs: remove broken lazytime support
  * fix multiplication overflow in copy_fdtable()
  * mtd: spinand: Propagate ECC information to the MTD structure
  * ima: Fix return value of ima_write_policy()
  * evm: Check also if *tfm is an error pointer in init_desc()
  * ima: Set file->f_mode instead of file->f_flags in ima_calc_file_hash()
  * riscv: set max_pfn to the PFN of the last page
  * KVM: SVM: Fix potential memory leak in svm_cpu_init()
  * i2c: dev: Fix the race between the release of i2c_dev and cdev
  * ubsan: build ubsan.c more conservatively
  * x86/uaccess, ubsan: Fix UBSAN vs. SMAP
  * ANDROID: scsi: ufs: Handle clocks when lrbp fails
  * f2fs: avoid inifinite loop to wait for flushing node pages at cp_error
  * ANDROID: fscrypt: handle direct I/O with IV_INO_LBLK_32
  * BACKPORT: FROMLIST: fscrypt: add support for IV_INO_LBLK_32 policies
  * ANDROID: Update the ABI xml and qcom whitelist
  * ANDROID: Fix build.config.gki-debug
  * Merge 4.19.124 into android-4.19-stable
  * Linux 4.19.124
  * Makefile: disallow data races on gcc-10 as well
  * KVM: x86: Fix off-by-one error in kvm_vcpu_ioctl_x86_setup_mce
  * ARM: dts: r8a7740: Add missing extal2 to CPG node
  * arm64: dts: renesas: r8a77980: Fix IPMMU VIP[01] nodes
  * ARM: dts: r8a73a4: Add missing CMT1 interrupts
  * arm64: dts: rockchip: Rename dwc3 device nodes on rk3399 to make dtc happy
  * arm64: dts: rockchip: Replace RK805 PMIC node name with "pmic" on rk3328 boards
  * clk: Unlink clock if failed to prepare or enable
  * Revert "ALSA: hda/realtek: Fix pop noise on ALC225"
  * usb: gadget: legacy: fix error return code in cdc_bind()
  * usb: gadget: legacy: fix error return code in gncm_bind()
  * usb: gadget: audio: Fix a missing error return value in audio_bind()
  * usb: gadget: net2272: Fix a memory leak in an error handling path in 'net2272_plat_probe()'
  * dwc3: Remove check for HWO flag in dwc3_gadget_ep_reclaim_trb_sg()
  * clk: rockchip: fix incorrect configuration of rk3228 aclk_gpu* clocks
  * exec: Move would_dump into flush_old_exec
  * x86/unwind/orc: Fix error handling in __unwind_start()
  * x86: Fix early boot crash on gcc-10, third try
  * cifs: fix leaked reference on requeued write
  * ARM: dts: imx27-phytec-phycard-s-rdk: Fix the I2C1 pinctrl entries
  * ARM: dts: dra7: Fix bus_dma_limit for PCIe
  * usb: xhci: Fix NULL pointer dereference when enqueuing trbs from urb sg list
  * USB: gadget: fix illegal array access in binding with UDC
  * usb: host: xhci-plat: keep runtime active when removing host
  * usb: core: hub: limit HUB_QUIRK_DISABLE_AUTOSUSPEND to USB5534B
  * ALSA: usb-audio: Add control message quirk delay for Kingston HyperX headset
  * ALSA: rawmidi: Fix racy buffer resize under concurrent accesses
  * ALSA: hda/realtek - Limit int mic boost for Thinkpad T530
  * gcc-10: avoid shadowing standard library 'free()' in crypto
  * gcc-10: disable 'restrict' warning for now
  * gcc-10: disable 'stringop-overflow' warning for now
  * gcc-10: disable 'array-bounds' warning for now
  * gcc-10: disable 'zero-length-bounds' warning for now
  * Stop the ad-hoc games with -Wno-maybe-initialized
  * kbuild: compute false-positive -Wmaybe-uninitialized cases in Kconfig
  * gcc-10 warnings: fix low-hanging fruit
  * pnp: Use list_for_each_entry() instead of open coding
  * hwmon: (da9052) Synchronize access with mfd
  * IB/mlx4: Test return value of calls to ib_get_cached_pkey
  * netfilter: nft_set_rbtree: Introduce and use nft_rbtree_interval_start()
  * arm64: fix the flush_icache_range arguments in machine_kexec
  * netfilter: conntrack: avoid gcc-10 zero-length-bounds warning
  * NFSv4: Fix fscache cookie aux_data to ensure change_attr is included
  * nfs: fscache: use timespec64 in inode auxdata
  * NFS: Fix fscache super_cookie index_key from changing after umount
  * mmc: block: Fix request completion in the CQE timeout path
  * mmc: core: Check request type before completing the request
  * i40iw: Fix error handling in i40iw_manage_arp_cache()
  * pinctrl: cherryview: Add missing spinlock usage in chv_gpio_irq_handler
  * pinctrl: baytrail: Enable pin configuration setting for GPIO chip
  * gfs2: Another gfs2_walk_metadata fix
  * ALSA: hda/realtek - Fix S3 pop noise on Dell Wyse
  * ipc/util.c: sysvipc_find_ipc() incorrectly updates position index
  * drm/qxl: lost qxl_bo_kunmap_atomic_page in qxl_image_init_helper()
  * ALSA: hda/hdmi: fix race in monitor detection during probe
  * cpufreq: intel_pstate: Only mention the BIOS disabling turbo mode once
  * dmaengine: mmp_tdma: Reset channel error on release
  * dmaengine: pch_dma.c: Avoid data race between probe and irq handler
  * riscv: fix vdso build with lld
  * tcp: fix SO_RCVLOWAT hangs with fat skbs
  * net: tcp: fix rx timestamp behavior for tcp_recvmsg
  * netprio_cgroup: Fix unlimited memory leak of v2 cgroups
  * net: ipv4: really enforce backoff for redirects
  * net: dsa: loop: Add module soft dependency
  * hinic: fix a bug of ndo_stop
  * virtio_net: fix lockdep warning on 32 bit
  * tcp: fix error recovery in tcp_zerocopy_receive()
  * Revert "ipv6: add mtu lock check in __ip6_rt_update_pmtu"
  * pppoe: only process PADT targeted at local interfaces
  * net: phy: fix aneg restart in phy_ethtool_set_eee
  * netlabel: cope with NULL catmap
  * net: fix a potential recursive NETDEV_FEAT_CHANGE
  * mmc: sdhci-acpi: Add SDHCI_QUIRK2_BROKEN_64_BIT_DMA for AMDI0040
  * scsi: sg: add sg_remove_request in sg_write
  * virtio-blk: handle block_device_operations callbacks after hot unplug
  * drop_monitor: work around gcc-10 stringop-overflow warning
  * net: moxa: Fix a potential double 'free_irq()'
  * net/sonic: Fix a resource leak in an error handling path in 'jazz_sonic_probe()'
  * shmem: fix possible deadlocks on shmlock_user_lock
  * net: dsa: Do not make user port errors fatal
  * ANDROID: rtc: class: call hctosys in resource managed registration
  * ANDROID: GKI: Update the ABI xml and whitelist
  * ANDROID: power_supply: Add RTX power-supply property
  * f2fs: compress: fix zstd data corruption
  * f2fs: add compressed/gc data read IO stat
  * f2fs: fix potential use-after-free issue
  * f2fs: compress: don't handle non-compressed data in workqueue
  * f2fs: remove redundant assignment to variable err
  * f2fs: refactor resize_fs to avoid meta updates in progress
  * f2fs: use round_up to enhance calculation
  * f2fs: introduce F2FS_IOC_RESERVE_COMPRESS_BLOCKS
  * f2fs: Avoid double lock for cp_rwsem during checkpoint
  * f2fs: report delalloc reserve as non-free in statfs for project quota
  * f2fs: Fix wrong stub helper update_sit_info
  * f2fs: compress: let lz4 compressor handle output buffer budget properly
  * f2fs: remove blk_plugging in block_operations
  * f2fs: introduce F2FS_IOC_RELEASE_COMPRESS_BLOCKS
  * f2fs: shrink spinlock coverage
  * f2fs: correctly fix the parent inode number during fsync()
  * f2fs: introduce mempool for {,de}compress intermediate page allocation
  * f2fs: introduce f2fs_bmap_compress()
  * f2fs: support fiemap on compressed inode
  * f2fs: support partial truncation on compressed inode
  * f2fs: remove redundant compress inode check
  * f2fs: use strcmp() in parse_options()
  * f2fs: Use the correct style for SPDX License Identifier
  * f2fs: flush dirty meta pages when flushing them
  * f2fs: fix checkpoint=disable:%u%%
  * f2fs: rework filename handling
  * f2fs: split f2fs_d_compare() from f2fs_match_name()
  * f2fs: don't leak filename in f2fs_try_convert_inline_dir()
  * ANDROID: clang: update to 11.0.1
  * FROMLIST: x86_64: fix jiffies ODR violation
  * ANDROID: arm64: vdso: Fix removing SCS flags
  * Merge branch 'android-4.19' into android-4.19-stable
  * ANDROID: GKI: Update the ABI xml and whitelist
  * ANDROID: Incremental fs: wake up log pollers less often
  * ANDROID: Incremental fs: Fix scheduling while atomic error
  * ANDROID: Incremental fs: Avoid continually recalculating hashes
  * ANDROID: export: Disable symbol trimming on modules
  * ANDROID: GKI: Update the ABI xml and whitelist
  * ANDROID: fscrypt: set dun_bytes more precisely
  * ANDROID: dm-default-key: set dun_bytes more precisely
  * ANDROID: block: backport the ability to specify max_dun_bytes
  * ANDROID: Revert "ANDROID: GKI: gki_defconfig: CONFIG_DM_DEFAULT_KEY=m"
  * msm: kgsl: Update GMU FW version for A619 GPU
  * Merge 4.19.123 into android-4.19
  * Linux 4.19.123
  * ipc/mqueue.c: change __do_notify() to bypass check_kill_permission()
  * scripts/decodecode: fix trapping instruction formatting
  * objtool: Fix stack offset tracking for indirect CFAs
  * netfilter: nf_osf: avoid passing pointer to local var
  * netfilter: nat: never update the UDP checksum when it's 0
  * x86/unwind/orc: Fix premature unwind stoppage due to IRET frames
  * x86/unwind/orc: Fix error path for bad ORC entry type
  * x86/unwind/orc: Prevent unwinding before ORC initialization
  * x86/unwind/orc: Don't skip the first frame for inactive tasks
  * x86/entry/64: Fix unwind hints in rewind_stack_do_exit()
  * x86/entry/64: Fix unwind hints in kernel exit path
  * x86/entry/64: Fix unwind hints in register clearing code
  * batman-adv: Fix refcnt leak in batadv_v_ogm_process
  * batman-adv: Fix refcnt leak in batadv_store_throughput_override
  * batman-adv: Fix refcnt leak in batadv_show_throughput_override
  * batman-adv: fix batadv_nc_random_weight_tq
  * KVM: VMX: Mark RCX, RDX and RSI as clobbered in vmx_vcpu_run()'s asm blob
  * KVM: VMX: Explicitly reference RCX as the vmx_vcpu pointer in asm blobs
  * coredump: fix crash when umh is disabled
  * staging: gasket: Check the return value of gasket_get_bar_index()
  * mm/page_alloc: fix watchdog soft lockups during set_zone_contiguous()
  * arm64: hugetlb: avoid potential NULL dereference
  * KVM: arm64: Fix 32bit PC wrap-around
  * KVM: arm: vgic: Fix limit condition when writing to GICD_I[CS]ACTIVER
  * tracing: Add a vmalloc_sync_mappings() for safe measure
  * USB: serial: garmin_gps: add sanity checking for data length
  * USB: uas: add quirk for LaCie 2Big Quadra
  * HID: usbhid: Fix race between usbhid_close() and usbhid_stop()
  * sctp: Fix bundling of SHUTDOWN with COOKIE-ACK
  * HID: wacom: Read HID_DG_CONTACTMAX directly for non-generic devices
  * net: stricter validation of untrusted gso packets
  * bnxt_en: Fix VF anti-spoof filter setup.
  * bnxt_en: Improve AER slot reset.
  * net/mlx5: Fix command entry leak in Internal Error State
  * net/mlx5: Fix forced completion access non initialized command entry
  * bnxt_en: Fix VLAN acceleration handling in bnxt_fix_features().
  * tipc: fix partial topology connection closure
  * sch_sfq: validate silly quantum values
  * sch_choke: avoid potential panic in choke_reset()
  * net: usb: qmi_wwan: add support for DW5816e
  * net_sched: sch_skbprio: add message validation to skbprio_change()
  * net/mlx4_core: Fix use of ENOSPC around mlx4_counter_alloc()
  * net: macsec: preserve ingress frame ordering
  * fq_codel: fix TCA_FQ_CODEL_DROP_BATCH_SIZE sanity checks
  * dp83640: reverse arguments to list_add_tail
  * vt: fix unicode console freeing with a common interface
  * tracing/kprobes: Fix a double initialization typo
  * USB: serial: qcserial: Add DW5816e support
  * ANDROID: usb: gadget: Add missing inline qualifier to stub functions
  * ANDROID: Drop ABI monitoring from KASAN build config
  * ANDROID: Rename build.config.gki.arch_kasan
  * ANDROID: GKI: Enable CONFIG_STATIC_USERMODEHELPER
  * ANDROID: dm-default-key: Update key size for wrapped keys
  * ANDROID: gki_defconfig: enable CONFIG_MMC_CRYPTO
  * ANDROID: mmc: MMC crypto API
  * ANDROID: GKI: Update the ABI xml and whitelist
  * ANDROID: GKI: add missing exports for cam_smmu_api.ko
  * Merge 4.19.122 into android-4.19
  * Linux 4.19.122
  * drm/atomic: Take the atomic toys away from X
  * cgroup, netclassid: remove double cond_resched
  * mac80211: add ieee80211_is_any_nullfunc()
  * platform/x86: GPD pocket fan: Fix error message when temp-limits are out of range
  * ALSA: hda: Match both PCI ID and SSID for driver blacklist
  * hexagon: define ioremap_uc
  * hexagon: clean up ioremap
  * mfd: intel-lpss: Use devm_ioremap_uc for MMIO
  * lib: devres: add a helper function for ioremap_uc
  * drm/amdgpu: Fix oops when pp_funcs is unset in ACPI event
  * sctp: Fix SHUTDOWN CTSN Ack in the peer restart case
  * net: systemport: suppress warnings on failed Rx SKB allocations
  * net: bcmgenet: suppress warnings on failed Rx SKB allocations
  * lib/mpi: Fix building for powerpc with clang
  * scripts/config: allow colons in option strings for sed
  * s390/ftrace: fix potential crashes when switching tracers
  * cifs: protect updating server->dstaddr with a spinlock
  * ASoC: rsnd: Fix "status check failed" spam for multi-SSI
  * ASoC: rsnd: Don't treat master SSI in multi SSI setup as parent
  * net: stmmac: Fix sub-second increment
  * net: stmmac: fix enabling socfpga's ptp_ref_clock
  * wimax/i2400m: Fix potential urb refcnt leak
  * drm/amdgpu: Correctly initialize thermal controller for GPUs with Powerplay table v0 (e.g Hawaii)
  * ASoC: codecs: hdac_hdmi: Fix incorrect use of list_for_each_entry
  * ASoC: rsnd: Fix HDMI channel mapping for multi-SSI mode
  * ASoC: rsnd: Fix parent SSI start/stop in multi-SSI mode
  * usb: dwc3: gadget: Properly set maxpacket limit
  * ASoC: sgtl5000: Fix VAG power-on handling
  * selftests/ipc: Fix test failure seen after initial test run
  * ASoC: topology: Check return value of pcm_new_ver
  * powerpc/pci/of: Parse unassigned resources
  * vhost: vsock: kick send_pkt worker once device is started
  * ANDROID: GKI: fix build warning on 32bits due to ASoC msm change
  * ANDROID: GKI: fix build error on 32bits due to ASoC msm change
  * ANDROID: GKI: update abi definition due to FAIR_GROUP_SCHED removal
  * ANDROID: GKI: Remove FAIR_GROUP_SCHED
  * ANDROID: GKI: BULK update ABI XML representation and qcom whitelist
  * ANDROID: build.config.gki.aarch64: Enable WHITELIST_STRICT_MODE
  * ANDROID: GKI: Update the ABI xml and qcom whitelist
  * ANDROID: remove unused variable
  * ANDROID: Drop ABI monitoring from KASAN build config
  * Merge 4.19.121 into android-4.19
  * Linux 4.19.121
  * mmc: meson-mx-sdio: remove the broken ->card_busy() op
  * mmc: meson-mx-sdio: Set MMC_CAP_WAIT_WHILE_BUSY
  * mmc: sdhci-msm: Enable host capabilities pertains to R1b response
  * mmc: sdhci-pci: Fix eMMC driver strength for BYT-based controllers
  * mmc: sdhci-xenon: fix annoying 1.8V regulator warning
  * mmc: cqhci: Avoid false "cqhci: CQE stuck on" by not open-coding timeout loop
  * btrfs: transaction: Avoid deadlock due to bad initialization timing of fs_info::journal_info
  * btrfs: fix partial loss of prealloc extent past i_size after fsync
  * selinux: properly handle multiple messages in selinux_netlink_send()
  * dmaengine: dmatest: Fix iteration non-stop logic
  * nfs: Fix potential posix_acl refcnt leak in nfs3_set_acl
  * ALSA: opti9xx: shut up gcc-10 range warning
  * iommu/amd: Fix legacy interrupt remapping for x2APIC-enabled system
  * scsi: target/iblock: fix WRITE SAME zeroing
  * iommu/qcom: Fix local_base status check
  * vfio/type1: Fix VA->PA translation for PFNMAP VMAs in vaddr_get_pfn()
  * vfio: avoid possible overflow in vfio_iommu_type1_pin_pages
  * RDMA/core: Fix race between destroy and release FD object
  * RDMA/core: Prevent mixed use of FDs between shared ufiles
  * RDMA/mlx4: Initialize ib_spec on the stack
  * RDMA/mlx5: Set GRH fields in query QP on RoCE
  * scsi: qla2xxx: check UNLOADING before posting async work
  * scsi: qla2xxx: set UNLOADING before waiting for session deletion
  * dm multipath: use updated MPATHF_QUEUE_IO on mapping for bio-based mpath
  * dm writecache: fix data corruption when reloading the target
  * dm verity fec: fix hash block number in verity_fec_decode
  * PM: hibernate: Freeze kernel threads in software_resume()
  * PM: ACPI: Output correct message on target power state
  * ALSA: pcm: oss: Place the plugin buffer overflow checks correctly
  * ALSA: hda/hdmi: fix without unlocked before return
  * ALSA: usb-audio: Correct a typo of NuPrime DAC-10 USB ID
  * ALSA: hda/realtek - Two front mics on a Lenovo ThinkCenter
  * btrfs: fix block group leak when removing fails
  * drm/qxl: qxl_release use after free
  * drm/qxl: qxl_release leak in qxl_hw_surface_alloc()
  * drm/qxl: qxl_release leak in qxl_draw_dirty_fb()
  * drm/edid: Fix off-by-one in DispID DTD pixel clock
  * ANDROID: GKI: Bulk update ABI XML representation
  * ANDROID: GKI: Enable net testing options
  * ANDROID: gki_defconfig: Enable CONFIG_REMOTEPROC
  * ANDROID: Rename build.config.gki.arch_kasan
  * ANDROID: GKI: Update ABI for IOMMU
  * ANDROID: Incremental fs: Fix issues with very large files
  * ANDROID: Correct build.config branch name
  * ANDROID: GKI: Bulk update ABI XML representation and whitelist.
  * UPSTREAM: vdso: Fix clocksource.h macro detection
  * Merge 4.19.120 into android-4.19
  * ANDROID: GKI: update abi definition due to added padding
  * ANDROID: GKI: networking: add Android ABI padding to a lot of networking structures
  * ANDROID: GKI: dma-mapping.h: add Android ABI padding to a structure
  * ANDROID: GKI: ioport.h: add Android ABI padding to a structure
  * ANDROID: GKI: iomap.h: add Android ABI padding to a structure
  * ANDROID: GKI: genhd.h: add Android ABI padding to some structures
  * ANDROID: GKI: hrtimer.h: add Android ABI padding to a structure
  * ANDROID: GKI: ethtool.h: add Android ABI padding to a structure
  * ANDROID: GKI: sched: add Android ABI padding to some structures
  * ANDROID: GKI: kernfs.h: add Android ABI padding to some structures
  * ANDROID: GKI: kobject.h: add Android ABI padding to some structures
  * ANDROID: GKI: mm.h: add Android ABI padding to a structure
  * ANDROID: GKI: mmu_notifier.h: add Android ABI padding to some structures
  * ANDROID: GKI: pci: add Android ABI padding to some structures
  * ANDROID: GKI: irqdomain.h: add Android ABI padding to a structure
  * ANDROID: GKI: blk_types.h: add Android ABI padding to a structure
  * ANDROID: GKI: scsi.h: add Android ABI padding to a structure
  * ANDROID: GKI: quota.h: add Android ABI padding to some structures
  * ANDROID: GKI: timer.h: add Android ABI padding to a structure
  * ANDROID: GKI: user_namespace.h: add Android ABI padding to a structure
  * FROMGIT: f2fs: fix missing check for f2fs_unlock_op
  * Linux 4.19.120
  * propagate_one(): mnt_set_mountpoint() needs mount_lock
  * ext4: check for non-zero journal inum in ext4_calculate_overhead
  * qed: Fix use after free in qed_chain_free
  * bpf, x86_32: Fix clobbering of dst for BPF_JSET
  * hwmon: (jc42) Fix name to have no illegal characters
  * ext4: convert BUG_ON's to WARN_ON's in mballoc.c
  * ext4: increase wait time needed before reuse of deleted inode numbers
  * ext4: use matching invalidatepage in ext4_writepage
  * arm64: Delete the space separator in __emit_inst
  * ALSA: hda: call runtime_allow() for all hda controllers
  * xen/xenbus: ensure xenbus_map_ring_valloc() returns proper grant status
  * objtool: Support Clang non-section symbols in ORC dump
  * objtool: Fix CONFIG_UBSAN_TRAP unreachable warnings
  * scsi: target: tcmu: reset_ring should reset TCMU_DEV_BIT_BROKEN
  * scsi: target: fix PR IN / READ FULL STATUS for FC
  * ALSA: hda: Explicitly permit using autosuspend if runtime PM is supported
  * ALSA: hda: Keep the controller initialization even if no codecs found
  * xfs: fix partially uninitialized structure in xfs_reflink_remap_extent
  * x86: hyperv: report value of misc_features
  * net: fec: set GPR bit on suspend by DT configuration.
  * bpf, x86: Fix encoding for lower 8-bit registers in BPF_STX BPF_B
  * xfs: clear PF_MEMALLOC before exiting xfsaild thread
  * mm: shmem: disable interrupt when acquiring info->lock in userfaultfd_copy path
  * bpf, x86_32: Fix incorrect encoding in BPF_LDX zero-extension
  * perf/core: fix parent pid/tid in task exit events
  * net/mlx5: Fix failing fw tracer allocation on s390
  * cpumap: Avoid warning when CONFIG_DEBUG_PER_CPU_MAPS is enabled
  * ARM: dts: bcm283x: Disable dsi0 node
  * PCI: Move Apex Edge TPU class quirk to fix BAR assignment
  * PCI: Avoid ASMedia XHCI USB PME# from D0 defect
  * svcrdma: Fix leak of svc_rdma_recv_ctxt objects
  * svcrdma: Fix trace point use-after-free race
  * xfs: acquire superblock freeze protection on eofblocks scans
  * net/cxgb4: Check the return from t4_query_params properly
  * rxrpc: Fix DATA Tx to disable nofrag for UDP on AF_INET6 socket
  * i2c: altera: use proper variable to hold errno
  * nfsd: memory corruption in nfsd4_lock()
  * ASoC: wm8960: Fix wrong clock after suspend & resume
  * ASoC: tas571x: disable regulators on failed probe
  * ASoC: q6dsp6: q6afe-dai: add missing channels to MI2S DAIs
  * iio:ad7797: Use correct attribute_group
  * usb: gadget: udc: bdc: Remove unnecessary NULL checks in bdc_req_complete
  * usb: dwc3: gadget: Do link recovery for SS and SSP
  * binder: take read mode of mmap_sem in binder_alloc_free_page()
  * include/uapi/linux/swab.h: fix userspace breakage, use __BITS_PER_LONG for swap
  * mtd: cfi: fix deadloop in cfi_cmdset_0002.c do_write_buffer
  * remoteproc: Fix wrong rvring index computation
  * FROMLIST: PM / devfreq: Restart previous governor if new governor fails to start
  * ANDROID: GKI: arm64: Enable GZIP and LZ4 kernel compression modes
  * ANDROID: GKI: arm64: gki_defconfig: Set arm_smmu configuration
  * ANDROID: GKI: iommu/arm-smmu: Modularize ARM SMMU driver
  * ANDROID: GKI: iommu: Snapshot of vendor changes
  * ANDROID: GKI: Additions to ARM SMMU register definitions
  * ANDROID: GKI: iommu/io-pgtable-arm: LPAE related updates by vendor
  * ANDROID: GKI: common: dma-mapping: make dma_common_contiguous_remap more robust
  * ANDROID: GKI: dma-coherent: Expose device base address and size
  * ANDROID: GKI: arm64: add support for NO_KERNEL_MAPPING and STRONGLY_ORDERED
  * ANDROID: GKI: dma-mapping: Add dma_remap functions
  * ANDROID: GKI: arm64: Support early fixup for CMA
  * ANDROID: GKI: iommu: dma-mapping-fast: Fast ARMv7/v8 Long Descriptor Format
  * ANDROID: GKI: arm64: dma-mapping: add support for IOMMU mapper
  * ANDROID: GKI: add ARCH_NR_GPIO for ABI match
  * ANDROID: GKI: kernel: Export symbol of `cpu_do_idle`
  * ANDROID: GKI: kernel: Export symbols needed by msm_minidump.ko and minidump_log.ko (again)
  * ANDROID: GKI: add missing exports for __flush_dcache_area
  * ANDROID: GKI: arm64: Export caching APIs
  * ANDROID: GKI: arm64: provide dma cache routines with same API as 32 bit
  * ANDROID: gki_defconfig: add FORTIFY_SOURCE, remove SPMI_MSM_PMIC_ARB
  * Revert "ANDROID: GKI: spmi: pmic-arb: don't enable SPMI_MSM_PMIC_ARB by default"
  * ANDROID: GKI: update abi definitions after adding padding
  * ANDROID: GKI: elevator: add Android ABI padding to some structures
  * ANDROID: GKI: dentry: add Android ABI padding to some structures
  * ANDROID: GKI: bio: add Android ABI padding to some structures
  * ANDROID: GKI: scsi: add Android ABI padding to some structures
  * ANDROID: GKI: ufs: add Android ABI padding to some structures
  * ANDROID: GKI: workqueue.h: add Android ABI padding to some structures
  * ANDROID: GKI: fs.h: add Android ABI padding to some structures
  * ANDROID: GKI: USB: add Android ABI padding to some structures
  * ANDROID: GKI: mm: add Android ABI padding to some structures
  * ANDROID: GKI: mount.h: add Android ABI padding to some structures
  * ANDROID: GKI: sched.h: add Android ABI padding to some structures
  * ANDROID: GKI: sock.h: add Android ABI padding to some structures
  * ANDROID: GKI: module.h: add Android ABI padding to some structures
  * ANDROID: GKI: device.h: add Android ABI padding to some structures
  * ANDROID: GKI: phy: add Android ABI padding to some structures
  * ANDROID: GKI: add android_kabi.h
  * ANDROID: ABI: update due to previous changes in the tree
  * BACKPORT: sched/core: Fix reset-on-fork from RT with uclamp
  * ANDROID: GKI: Add support for missing V4L2 symbols
  * ANDROID: GKI: Bulk update ABI XML representation
  * ANDROID: GKI: arm64: psci: Support for OS initiated scheme
  * ANDROID: GKI: net: add counter for number of frames coalesced in GRO
  * ANDROID: GKI: cfg80211: Include length of kek in rekey data
  * BACKPORT: loop: change queue block size to match when using DIO
  * ANDROID: Incremental fs: Add setattr call
  * ANDROID: GKI: enable CONFIG_RTC_SYSTOHC
  * ANDROID: GKI: ipv4: add vendor padding to __IPV4_DEVCONF_* enums
  * Revert "ANDROID: GKI: ipv4: increase __IPV4_DEVCONF_MAX to 64"
  * ANDROID: driver: gpu: drm: fix export symbol types
  * ANDROID: SoC: core: fix export symbol type
  * ANDROID: ufshcd-crypto: fix export symbol type
  * ANDROID: GKI: drivers: mailbox: fix race resulting in multiple message submission
  * ANDROID: GKI: arm64: gki_defconfig: Enable a few thermal configs
  * Revert "ANDROID: GKI: add base.h include to match MODULE_VERSIONS"
  * FROMLIST: thermal: Make cooling device trip point writable from sysfs
  * ANDROID: GKI: drivers: thermal: cpu_cooling: Use CPU ID as cooling device ID
  * ANDROID: GKI: PM / devfreq: Allow min freq to be 0
  * ANDROID: GKI: arm64: gki_defconfig: Enable REGULATOR_PROXY_CONSUMER
  * ANDROID: GKI: Bulk Update ABI XML representation
  * ANDROID: KASAN support for GKI remove CONFIG_CC_WERROR
  * ANDROID: KASAN support for GKI
  * ANDROID: virt_wifi: fix export symbol types
  * ANDROID: vfs: fix export symbol type
  * ANDROID: vfs: fix export symbol types
  * ANDROID: fscrypt: fix export symbol type
  * ANDROID: cfi: fix export symbol types
  * ANDROID: bpf: fix export symbol type
  * Merge 4.19.119 into android-4.19
  * Linux 4.19.119
  * s390/mm: fix page table upgrade vs 2ndary address mode accesses
  * xfs: Fix deadlock between AGI and AGF with RENAME_WHITEOUT
  * serial: sh-sci: Make sure status register SCxSR is read in correct sequence
  * xhci: prevent bus suspend if a roothub port detected a over-current condition
  * usb: f_fs: Clear OS Extended descriptor counts to zero in ffs_data_reset()
  * usb: dwc3: gadget: Fix request completion check
  * UAS: fix deadlock in error handling and PM flushing work
  * UAS: no use logging any details in case of ENODEV
  * cdc-acm: introduce a cool down
  * cdc-acm: close race betrween suspend() and acm_softint
  * staging: vt6656: Power save stop wake_up_count wrap around.
  * staging: vt6656: Fix pairwise key entry save.
  * staging: vt6656: Fix drivers TBTT timing counter.
  * staging: vt6656: Fix calling conditions of vnt_set_bss_mode
  * staging: vt6656: Don't set RCR_MULTICAST or RCR_BROADCAST by default.
  * vt: don't use kmalloc() for the unicode screen buffer
  * vt: don't hardcode the mem allocation upper bound
  * staging: comedi: Fix comedi_device refcnt leak in comedi_open
  * staging: comedi: dt2815: fix writing hi byte of analog output
  * powerpc/setup_64: Set cache-line-size based on cache-block-size
  * ARM: imx: provide v7_cpu_resume() only on ARM_CPU_SUSPEND=y
  * iwlwifi: mvm: beacon statistics shouldn't go backwards
  * iwlwifi: pcie: actually release queue memory in TVQM
  * ASoC: dapm: fixup dapm kcontrol widget
  * audit: check the length of userspace generated audit records
  * usb-storage: Add unusual_devs entry for JMicron JMS566
  * tty: rocket, avoid OOB access
  * tty: hvc: fix buffer overflow during hvc_alloc().
  * KVM: VMX: Enable machine check support for 32bit targets
  * KVM: Check validity of resolved slot when searching memslots
  * KVM: s390: Return last valid slot if approx index is out-of-bounds
  * tpm: ibmvtpm: retry on H_CLOSED in tpm_ibmvtpm_send()
  * tpm/tpm_tis: Free IRQ if probing fails
  * ALSA: usb-audio: Filter out unsupported sample rates on Focusrite devices
  * ALSA: usb-audio: Fix usb audio refcnt leak when getting spdif
  * ALSA: hda/realtek - Add new codec supported for ALC245
  * ALSA: hda/realtek - Fix unexpected init_amp override
  * ALSA: usx2y: Fix potential NULL dereference
  * tools/vm: fix cross-compile build
  * mm/ksm: fix NULL pointer dereference when KSM zero page is enabled
  * mm/hugetlb: fix a addressing exception caused by huge_pte_offset
  * vmalloc: fix remap_vmalloc_range() bounds checks
  * USB: hub: Fix handling of connect changes during sleep
  * USB: core: Fix free-while-in-use bug in the USB S-Glibrary
  * USB: early: Handle AMD's spec-compliant identifiers, too
  * USB: Add USB_QUIRK_DELAY_CTRL_MSG and USB_QUIRK_DELAY_INIT for Corsair K70 RGB RAPIDFIRE
  * USB: sisusbvga: Change port variable from signed to unsigned
  * fs/namespace.c: fix mountpoint reference counter race
  * iio: xilinx-xadc: Make sure not exceed maximum samplerate
  * iio: xilinx-xadc: Fix sequencer configuration for aux channels in simultaneous mode
  * iio: xilinx-xadc: Fix clearing interrupt when enabling trigger
  * iio: xilinx-xadc: Fix ADC-B powerdown
  * iio: adc: stm32-adc: fix sleep in atomic context
  * iio: st_sensors: rely on odr mask to know if odr can be set
  * iio: core: remove extra semi-colon from devm_iio_device_register() macro
  * ALSA: usb-audio: Add connector notifier delegation
  * ALSA: usb-audio: Add static mapping table for ALC1220-VB-based mobos
  * ALSA: hda: Remove ASUS ROG Zenith from the blacklist
  * KEYS: Avoid false positive ENOMEM error on key read
  * mlxsw: Fix some IS_ERR() vs NULL bugs
  * vrf: Check skb for XFRM_TRANSFORMED flag
  * xfrm: Always set XFRM_TRANSFORMED in xfrm{4,6}_output_finish
  * net: dsa: b53: b53_arl_rw_op() needs to select IVL or SVL
  * net: dsa: b53: Rework ARL bin logic
  * net: dsa: b53: Fix ARL register definitions
  * net: dsa: b53: Lookup VID in ARL searches when VLAN is enabled
  * vrf: Fix IPv6 with qdisc and xfrm
  * team: fix hang in team_mode_get()
  * tcp: cache line align MAX_TCP_HEADER
  * sched: etf: do not assume all sockets are full blown
  * net/x25: Fix x25_neigh refcnt leak when receiving frame
  * net: stmmac: dwmac-meson8b: Add missing boundary to RGMII TX clock array
  * net: netrom: Fix potential nr_neigh refcnt leak in nr_add_node
  * net: bcmgenet: correct per TX/RX ring statistics
  * macvlan: fix null dereference in macvlan_device_event()
  * macsec: avoid to set wrong mtu
  * ipv6: fix restrict IPV6_ADDRFORM operation
  * cxgb4: fix large delays in PTP synchronization
  * cxgb4: fix adapter crash due to wrong MC size
  * x86/KVM: Clean up host's steal time structure
  * x86/KVM: Make sure KVM_VCPU_FLUSH_TLB flag is not missed
  * x86/kvm: Cache gfn to pfn translation
  * x86/kvm: Introduce kvm_(un)map_gfn()
  * KVM: Properly check if "page" is valid in kvm_vcpu_unmap
  * kvm: fix compile on s390 part 2
  * kvm: fix compilation on s390
  * kvm: fix compilation on aarch64
  * KVM: Introduce a new guest mapping API
  * KVM: nVMX: Always sync GUEST_BNDCFGS when it comes from vmcs01
  * KVM: VMX: Zero out *all* general purpose registers after VM-Exit
  * f2fs: fix to avoid memory leakage in f2fs_listxattr
  * blktrace: fix dereference after null check
  * blktrace: Protect q->blk_trace with RCU
  * net: ipv6_stub: use ip6_dst_lookup_flow instead of ip6_dst_lookup
  * net: ipv6: add net argument to ip6_dst_lookup_flow
  * PCI/ASPM: Allow re-enabling Clock PM
  * scsi: smartpqi: fix call trace in device discovery
  * virtio-blk: improve virtqueue error to BLK_STS
  * tracing/selftests: Turn off timeout setting
  * drm/amd/display: Not doing optimize bandwidth if flip pending.
  * xhci: Ensure link state is U3 after setting USB_SS_PORT_LS_U3
  * ASoC: Intel: bytcr_rt5640: Add quirk for MPMAN MPWIN895CL tablet
  * perf/core: Disable page faults when getting phys address
  * pwm: bcm2835: Dynamically allocate base
  * pwm: renesas-tpu: Fix late Runtime PM enablement
  * Revert "powerpc/64: irq_work avoid interrupt when called with hardware irqs enabled"
  * loop: Better discard support for block devices
  * s390/cio: avoid duplicated 'ADD' uevents
  * kconfig: qconf: Fix a few alignment issues
  * ipc/util.c: sysvipc_find_ipc() should increase position index
  * selftests: kmod: fix handling test numbers above 9
  * kernel/gcov/fs.c: gcov_seq_next() should increase position index
  * nvme: fix deadlock caused by ANA update wrong locking
  * ASoC: Intel: atom: Take the drv->lock mutex before calling sst_send_slot_map()
  * scsi: iscsi: Report unbind session event when the target has been removed
  * pwm: rcar: Fix late Runtime PM enablement
  * ceph: don't skip updating wanted caps when cap is stale
  * ceph: return ceph_mdsc_do_request() errors from __get_parent()
  * scsi: lpfc: Fix crash in target side cable pulls hitting WAIT_FOR_UNREG
  * scsi: lpfc: Fix kasan slab-out-of-bounds error in lpfc_unreg_login
  * watchdog: reset last_hw_keepalive time at start
  * arm64: Silence clang warning on mismatched value/register sizes
  * arm64: compat: Workaround Neoverse-N1 #1542419 for compat user-space
  * arm64: Fake the IminLine size on systems affected by Neoverse-N1 #1542419
  * arm64: errata: Hide CTR_EL0.DIC on systems affected by Neoverse-N1 #1542419
  * arm64: Add part number for Neoverse N1
  * vti4: removed duplicate log message.
  * crypto: mxs-dcp - make symbols 'sha1_null_hash' and 'sha256_null_hash' static
  * bpftool: Fix printing incorrect pointer in btf_dump_ptr
  * drm/msm: Use the correct dma_sync calls harder
  * ext4: fix extent_status fragmentation for plain files
  * ANDROID: abi_gki_aarch64_cuttlefish_whitelist: remove stale symbols
  * ANDROID: GKI: ipv4: increase __IPV4_DEVCONF_MAX to 64
  * ANDROID: GKI: power: add missing export for POWER_RESET_QCOM=m
  * BACKPORT: cfg80211: Support key configuration for Beacon protection (BIGTK)
  * BACKPORT: cfg80211: Enhance the AKM advertizement to support per interface.
  * UPSTREAM: sysrq: Use panic() to force a crash
  * ANDROID: GKI: kernel: sound: update codec options with block size
  * ANDROID: add compat cross compiler
  * ANDROID: x86/vdso: disable LTO only for VDSO
  * BACKPORT: arm64: vdso32: Enable Clang Compilation
  * UPSTREAM: arm64: compat: vdso: Expose BUILD_VDSO32
  * BACKPORT: lib/vdso: Enable common headers
  * BACKPORT: arm: vdso: Enable arm to use common headers
  * BACKPORT: x86/vdso: Enable x86 to use common headers
  * BACKPORT: mips: vdso: Enable mips to use common headers
  * UPSTREAM: arm64: vdso32: Include common headers in the vdso library
  * UPSTREAM: arm64: vdso: Include common headers in the vdso library
  * UPSTREAM: arm64: Introduce asm/vdso/processor.h
  * BACKPORT: arm64: vdso32: Code clean up
  * UPSTREAM: linux/elfnote.h: Replace elf.h with UAPI equivalent
  * UPSTREAM: scripts: Fix the inclusion order in modpost
  * UPSTREAM: common: Introduce processor.h
  * UPSTREAM: linux/ktime.h: Extract common header for vDSO
  * UPSTREAM: linux/jiffies.h: Extract common header for vDSO
  * UPSTREAM: linux/time64.h: Extract common header for vDSO
  * BACKPORT: linux/time32.h: Extract common header for vDSO
  * BACKPORT: linux/time.h: Extract common header for vDSO
  * UPSTREAM: linux/math64.h: Extract common header for vDSO
  * BACKPORT: linux/clocksource.h: Extract common header for vDSO
  * BACKPORT: mips: Introduce asm/vdso/clocksource.h
  * BACKPORT: arm64: Introduce asm/vdso/clocksource.h
  * BACKPORT: arm: Introduce asm/vdso/clocksource.h
  * BACKPORT: x86: Introduce asm/vdso/clocksource.h
  * UPSTREAM: linux/limits.h: Extract common header for vDSO
  * BACKPORT: linux/kernel.h: split *_MAX and *_MIN macros into <linux/limits.h>
  * BACKPORT: linux/bits.h: Extract common header for vDSO
  * UPSTREAM: linux/const.h: Extract common header for vDSO
  * BACKPORT: arm64: vdso: fix flip/flop vdso build bug
  * UPSTREAM: lib/vdso: Allow the high resolution parts to be compiled out
  * UPSTREAM: lib/vdso: Only read hrtimer_res when needed in __cvdso_clock_getres()
  * UPSTREAM: lib/vdso: Mark do_hres() and do_coarse() as __always_inline
  * UPSTREAM: lib/vdso: Avoid duplication in __cvdso_clock_getres()
  * UPSTREAM: lib/vdso: Let do_coarse() return 0 to simplify the callsite
  * UPSTREAM: lib/vdso: Remove checks on return value for 32 bit vDSO
  * UPSTREAM: lib/vdso: Build 32 bit specific functions in the right context
  * UPSTREAM: lib/vdso: Make __cvdso_clock_getres() static
  * UPSTREAM: lib/vdso: Make clock_getres() POSIX compliant again
  * UPSTREAM: lib/vdso/32: Provide legacy syscall fallbacks
  * UPSTREAM: lib/vdso: Move fallback invocation to the callers
  * UPSTREAM: lib/vdso/32: Remove inconsistent NULL pointer checks
  * UPSTREAM: lib/vdso: Make delta calculation work correctly
  * UPSTREAM: arm64: compat: Fix syscall number of compat_clock_getres
  * BACKPORT: arm64: lse: Fix LSE atomics with LLVM
  * UPSTREAM: mips: Fix gettimeofday() in the vdso library
  * UPSTREAM: mips: vdso: Fix __arch_get_hw_counter()
  * BACKPORT: arm64: Kconfig: Make CONFIG_COMPAT_VDSO a proper Kconfig option
  * UPSTREAM: arm64: vdso32: Rename COMPATCC to CC_COMPAT
  * UPSTREAM: arm64: vdso32: Pass '--target' option to clang via VDSO_CAFLAGS
  * UPSTREAM: arm64: vdso32: Don't use KBUILD_CPPFLAGS unconditionally
  * UPSTREAM: arm64: vdso32: Move definition of COMPATCC into vdso32/Makefile
  * UPSTREAM: arm64: Default to building compat vDSO with clang when CONFIG_CC_IS_CLANG
  * UPSTREAM: lib: vdso: Remove CROSS_COMPILE_COMPAT_VDSO
  * UPSTREAM: arm64: vdso32: Remove jump label config option in Makefile
  * UPSTREAM: arm64: vdso32: Detect binutils support for dmb ishld
  * BACKPORT: arm64: vdso: Remove stale files from old assembly implementation
  * UPSTREAM: arm64: vdso32: Fix broken compat vDSO build warnings
  * UPSTREAM: mips: compat: vdso: Use legacy syscalls as fallback
  * BACKPORT: arm64: Relax Documentation/arm64/tagged-pointers.rst
  * BACKPORT: arm64: Add tagged-address-abi.rst to index.rst
  * UPSTREAM: arm64: vdso: Fix Makefile regression
  * UPSTREAM: mips: vdso: Fix flip/flop vdso building bug
  * UPSTREAM: mips: vdso: Fix source path
  * UPSTREAM: mips: Add clock_gettime64 entry point
  * UPSTREAM: mips: Add clock_getres entry point
  * BACKPORT: mips: Add support for generic vDSO
  * BACKPORT: arm64: vdso: Explicitly add build-id option
  * BACKPORT: arm64: vdso: use $(LD) instead of $(CC) to link VDSO
  * BACKPORT: arm64: vdso: Cleanup Makefiles
  * UPSTREAM: arm64: vdso: Fix population of AT_SYSINFO_EHDR for compat vdso
  * UPSTREAM: arm64: vdso: Fix compilation with clang older than 8
  * UPSTREAM: arm64: compat: Fix __arch_get_hw_counter() implementation
  * UPSTREAM: arm64: Fix __arch_get_hw_counter() implementation
  * UPSTREAM: x86/vdso/32: Use 32bit syscall fallback
  * UPSTREAM: x86/vdso: Fix flip/flop vdso build bug
  * UPSTREAM: x86/vdso: Give the [ph]vclock_page declarations real types
  * UPSTREAM: x86/vdso: Add clock_gettime64() entry point
  * BACKPORT: x86/vdso: Add clock_getres() entry point
  * BACKPORT: x86/vdso: Switch to generic vDSO implementation
  * UPSTREAM: x86/segments: Introduce the 'CPUNODE' naming to better document the segment limit CPU/node NR trick
  * UPSTREAM: x86/vdso: Initialize the CPU/node NR segment descriptor earlier
  * UPSTREAM: x86/vdso: Introduce helper functions for CPU and node number
  * UPSTREAM: x86/segments/64: Rename the GDT PER_CPU entry to CPU_NUMBER
  * BACKPORT: arm64: vdso: Enable vDSO compat support
  * UPSTREAM: arm64: compat: Get sigreturn trampolines from vDSO
  * UPSTREAM: arm64: elf: VDSO code page discovery
  * UPSTREAM: arm64: compat: VDSO setup for compat layer
  * UPSTREAM: arm64: vdso: Refactor vDSO code
  * BACKPORT: arm64: compat: Add vDSO
  * UPSTREAM: arm64: compat: Generate asm offsets for signals
  * UPSTREAM: arm64: compat: Expose signal related structures
  * UPSTREAM: arm64: compat: Add missing syscall numbers
  * BACKPORT: arm64: vdso: Substitute gettimeofday() with C implementation
  * UPSTREAM: timekeeping: Provide a generic update_vsyscall() implementation
  * UPSTREAM: lib/vdso: Add compat support
  * UPSTREAM: lib/vdso: Provide generic VDSO implementation
  * UPSTREAM: vdso: Define standardized vdso_datapage
  * UPSTREAM: hrtimer: Split out hrtimer defines into separate header
  * UPSTREAM: nds32: Fix vDSO clock_getres()
  * UPSTREAM: arm64: compat: Reduce address limit for 64K pages
  * BACKPORT: arm64: compat: Add KUSER_HELPERS config option
  * UPSTREAM: arm64: compat: Refactor aarch32_alloc_vdso_pages()
  * BACKPORT: arm64: compat: Split kuser32
  * UPSTREAM: arm64: compat: Alloc separate pages for vectors and sigpage
  * ANDROID: GKI: Update ABI XML representation
  * ANDROID: GKI: Enable GENERIC_IRQ_CHIP
  * ANDROID: GKI: power_supply: Add FG_TYPE power-supply property
  * ANDROID: GKI: mm: export mm_trace_rss_stat for modules to report RSS changes
  * ANDROID: GKI: gki_defconfig: Enable CONFIG_LEDS_TRIGGER_TRANSIENT
  * ANDROID: GKI: gki_defconfig: Enable CONFIG_CPU_FREQ_STAT
  * ANDROID: GKI: arm64: gki_defconfig: Disable HW tracing features
  * ANDROID: GKI: gki_defconfig: Enable CONFIG_I2C_CHARDEV
  * ANDROID: Incremental fs: Use simple compression in log buffer
  * ANDROID: GKI: usb: core: Add support to parse config summary capability descriptors
  * ANDROID: GKI: Update ABI XML representation
  * ANDROID: dm-bow: Fix not to skip trim at framented range
  * ANDROID: Remove VLA from uid_sys_stats.c
  * f2fs: fix missing check for f2fs_unlock_op
  * ANDROID: fix wakeup reason findings
  * UPSTREAM: cfg80211: fix and clean up cfg80211_gen_new_bssid()
  * UPSTREAM: cfg80211: save multi-bssid properties
  * UPSTREAM: cfg80211: make BSSID generation function inline
  * UPSTREAM: cfg80211: parse multi-bssid only if HW supports it
  * UPSTREAM: cfg80211: Move Multiple BSS info to struct cfg80211_bss to be visible
  * UPSTREAM: cfg80211: Properly track transmitting and non-transmitting BSS
  * UPSTREAM: cfg80211: use for_each_element() for multi-bssid parsing
  * UPSTREAM: cfg80211: Parsing of Multiple BSSID information in scanning
  * UPSTREAM: cfg80211/nl80211: Offload OWE processing to user space in AP mode
  * ANDROID: GKI: cfg80211: Sync nl80211 commands/feature with upstream
  * ANDROID: GKI: gki_defconfig: Enable FW_LOADER_USER_HELPER*
  * ANDROID: GKI: arm64: gki_defconfig: Disable CONFIG_ARM64_TAGGED_ADDR_ABI
  * ANDROID: GKI: gki_defconfig: CONFIG_CHR_DEV_SG=y
  * ANDROID: GKI: gki_defconfig: CONFIG_DM_DEFAULT_KEY=m
  * ANDROID: update the ABI xml representation
  * ANDROID: init: GKI: enable hidden configs for GPU
  * Merge 4.19.118 into android-4.19
  * Linux 4.19.118
  * bpf: fix buggy r0 retval refinement for tracing helpers
  * KEYS: Don't write out to userspace while holding key semaphore
  * mtd: phram: fix a double free issue in error path
  * mtd: lpddr: Fix a double free in probe()
  * mtd: spinand: Explicitly use MTD_OPS_RAW to write the bad block marker to OOB
  * locktorture: Print ratio of acquisitions, not failures
  * tty: evh_bytechan: Fix out of bounds accesses
  * iio: si1133: read 24-bit signed integer for measurement
  * fbdev: potential information leak in do_fb_ioctl()
  * net: dsa: bcm_sf2: Fix overflow checks
  * f2fs: fix to wait all node page writeback
  * iommu/amd: Fix the configuration of GCR3 table root pointer
  * libnvdimm: Out of bounds read in __nd_ioctl()
  * power: supply: axp288_fuel_gauge: Broaden vendor check for Intel Compute Sticks.
  * ext2: fix debug reference to ext2_xattr_cache
  * ext2: fix empty body warnings when -Wextra is used
  * iommu/vt-d: Fix mm reference leak
  * drm/vc4: Fix HDMI mode validation
  * f2fs: fix NULL pointer dereference in f2fs_write_begin()
  * NFS: Fix memory leaks in nfs_pageio_stop_mirroring()
  * drm/amdkfd: kfree the wrong pointer
  * x86: ACPI: fix CPU hotplug deadlock
  * KVM: s390: vsie: Fix possible race when shadowing region 3 tables
  * compiler.h: fix error in BUILD_BUG_ON() reporting
  * percpu_counter: fix a data race at vm_committed_as
  * include/linux/swapops.h: correct guards for non_swap_entry()
  * cifs: Allocate encryption header through kmalloc
  * um: ubd: Prevent buffer overrun on command completion
  * ext4: do not commit super on read-only bdev
  * s390/cpum_sf: Fix wrong page count in error message
  * powerpc/maple: Fix declaration made after definition
  * s390/cpuinfo: fix wrong output when CPU0 is offline
  * NFS: direct.c: Fix memory leak of dreq when nfs_get_lock_context fails
  * NFSv4/pnfs: Return valid stateids in nfs_layout_find_inode_by_stateid()
  * rtc: 88pm860x: fix possible race condition
  * soc: imx: gpc: fix power up sequencing
  * clk: tegra: Fix Tegra PMC clock out parents
  * power: supply: bq27xxx_battery: Silence deferred-probe error
  * clk: at91: usb: continue if clk_hw_round_rate() return zero
  * x86/Hyper-V: Report crash data in die() when panic_on_oops is set
  * x86/Hyper-V: Report crash register data when sysctl_record_panic_msg is not set
  * x86/Hyper-V: Trigger crash enlightenment only once during system crash.
  * x86/Hyper-V: Free hv_panic_page when fail to register kmsg dump
  * x86/Hyper-V: Unload vmbus channel in hv panic callback
  * xsk: Add missing check on user supplied headroom size
  * rbd: call rbd_dev_unprobe() after unwatching and flushing notifies
  * rbd: avoid a deadlock on header_rwsem when flushing notifies
  * video: fbdev: sis: Remove unnecessary parentheses and commented code
  * lib/raid6: use vdupq_n_u8 to avoid endianness warnings
  * x86/Hyper-V: Report crash register data or kmsg before running crash kernel
  * of: overlay: kmemleak in dup_and_fixup_symbol_prop()
  * of: unittest: kmemleak in of_unittest_overlay_high_level()
  * of: unittest: kmemleak in of_unittest_platform_populate()
  * of: unittest: kmemleak on changeset destroy
  * ALSA: hda: Don't release card at firmware loading error
  * irqchip/mbigen: Free msi_desc on device teardown
  * netfilter: nf_tables: report EOPNOTSUPP on unsupported flags/object type
  * ARM: dts: imx6: Use gpc for FEC interrupt controller to fix wake on LAN.
  * arm, bpf: Fix bugs with ALU64 {RSH, ARSH} BPF_K shift by 0
  * watchdog: sp805: fix restart handler
  * ext4: use non-movable memory for superblock readahead
  * scsi: sg: add sg_remove_request in sg_common_write
  * objtool: Fix switch table detection in .text.unlikely
  * arm, bpf: Fix offset overflow for BPF_MEM BPF_DW
  * ANDROID: GKI: Bulk update ABI report.
  * ANDROID: GKI: qos: Register irq notify after adding the qos request
  * ANDROID: GKI: Add dual role mode to usb_dr_modes array
  * UPSTREAM: virtio-gpu api: comment feature flags
  * ANDROID: arch:arm64: Increase kernel command line size
  * ANDROID: GKI: Add special linux_banner_ptr for modules
  * Revert "ANDROID: GKI: Make linux_banner a C pointer"
  * ANDROID: GKI: PM / devfreq: Add new flag to do simple clock scaling
  * ANDROID: GKI: Resolve ABI diff for struct snd_usb_audio
  * ANDROID: GKI: Bulk update ABI
  * ANDROID: GKI: Update the whitelist for qcom SoCs
  * ANDROID: GKI: arm64: gki_defconfig: Set CONFIG_SCSI_UFSHCD=m
  * ANDROID: GKI: scsi: add option to override the command timeout
  * ANDROID: GKI: scsi: Adjust DBD setting in mode sense for caching mode page per LLD
  * ANDROID: add ion_stat tracepoint to common kernel
  * UPSTREAM: gpu/trace: add a gpu total memory usage tracepoint
  * Merge 4.19.117 into android-4.19
  * Linux 4.19.117
  * mm/vmalloc.c: move 'area->pages' after if statement
  * wil6210: remove reset file from debugfs
  * wil6210: make sure Rx ring sizes are correlated
  * wil6210: add general initialization/size checks
  * wil6210: ignore HALP ICR if already handled
  * wil6210: check rx_buff_mgmt before accessing it
  * x86/resctrl: Fix invalid attempt at removing the default resource group
  * x86/resctrl: Preserve CDP enable over CPU hotplug
  * x86/microcode/AMD: Increase microcode PATCH_MAX_SIZE
  * scsi: target: fix hang when multiple threads try to destroy the same iscsi session
  * scsi: target: remove boilerplate code
  * kvm: x86: Host feature SSBD doesn't imply guest feature SPEC_CTRL_SSBD
  * ext4: do not zeroout extents beyond i_disksize
  * drm/amd/powerplay: force the trim of the mclk dpm_levels if OD is enabled
  * usb: dwc3: gadget: Don't clear flags before transfer ended
  * usb: dwc3: gadget: don't enable interrupt when disabling endpoint
  * mac80211_hwsim: Use kstrndup() in place of kasprintf()
  * btrfs: check commit root generation in should_ignore_root
  * tracing: Fix the race between registering 'snapshot' event trigger and triggering 'snapshot' operation
  * keys: Fix proc_keys_next to increase position index
  * ALSA: usb-audio: Check mapping at creating connector controls, too
  * ALSA: usb-audio: Don't create jack controls for PCM terminals
  * ALSA: usb-audio: Don't override ignore_ctl_error value from the map
  * ALSA: usb-audio: Filter error from connector kctl ops, too
  * ASoC: Intel: mrfld: return error codes when an error occurs
  * ASoC: Intel: mrfld: fix incorrect check on p->sink
  * ext4: fix incorrect inodes per group in error message
  * ext4: fix incorrect group count in ext4_fill_super error message
  * pwm: pca9685: Fix PWM/GPIO inter-operation
  * jbd2: improve comments about freeing data buffers whose page mapping is NULL
  * scsi: ufs: Fix ufshcd_hold() caused scheduling while atomic
  * ovl: fix value of i_ino for lower hardlink corner case
  * net: dsa: mt7530: fix tagged frames pass-through in VLAN-unaware mode
  * net: stmmac: dwmac-sunxi: Provide TX and RX fifo sizes
  * net: revert default NAPI poll timeout to 2 jiffies
  * net: qrtr: send msgs from local of same id as broadcast
  * net: ipv6: do not consider routes via gateways for anycast address check
  * net: ipv4: devinet: Fix crash when add/del multicast IP with autojoin
  * hsr: check protocol version in hsr_newlink()
  * amd-xgbe: Use __napi_schedule() in BH context
  * ANDROID: GKI: drivers: of-thermal: Relate thermal zones using same sensor
  * ANDROID: GKI: Bulk ABI update
  * ANDROID: GKI: dma: Add set_dma_mask hook to struct dma_map_ops
  * Merge remote-tracking branch 'aosp/upstream-f2fs-stable-linux-4.19.y' into android-4.19
  * ANDROID: GKI: ABI update due to recent patches
  * Merge 4.19.116 into android-4.19
  * FROMLIST: drm/prime: add support for virtio exported objects
  * FROMLIST: dma-buf: add support for virtio exported objects
  * UPSTREAM: drm/virtio: module_param_named() requires linux/moduleparam.h
  * UPSTREAM: drm/virtio: fix resource id creation race
  * UPSTREAM: drm/virtio: make resource id workaround runtime switchable.
  * BACKPORT: drm/virtio: Drop deprecated load/unload initialization
  * ANDROID: GKI: Add DRM_TTM config to GKI
  * ANDROID: Bulk update the ABI xml representation
  * ANDROID: GKI: spmi: pmic-arb: don't enable SPMI_MSM_PMIC_ARB by default
  * ANDROID: GKI: attribute page lock and waitqueue functions as sched
  * ANDROID: GKI: extcon: Fix Add usage of blocking notifier chain
  * ANDROID: GKI: USB: pd: Extcon fix for C current
  * ANDROID: drm/dsi: Fix byte order of DCS set/get brightness
  * ANDROID: GKI: mm: Export symbols to modularize CONFIG_MSM_DRM
  * ANDROID: GKI: ALSA: compress: Add support to send codec specific data
  * ANDROID: GKI: ALSA: Compress - dont use lock for all ioctls
  * ANDROID: GKI: ASoC: msm: qdsp6v2: add support for AMR_WB_PLUS offload
  * ANDROID: GKI: msm: dolby: MAT and THD audiocodec name modification
  * ANDROID: GKI: asoc: msm: Add support for compressed perf mode
  * ANDROID: GKI: msm: audio: support for gapless_pcm
  * ANDROID: GKI: uapi: msm: dolby: Support for TrueHD and MAT decoders
  * ANDROID: GKI: ASoC: msm: qdsp6v2: Add TrueHD HDMI compress pass-though
  * ANDROID: GKI: ALSA: compress: Add APTX format support in ALSA
  * ANDROID: GKI: msm: qdsp6v2: Add timestamp support for compress capture
  * ANDROID: GKI: SoC: msm: Add support for meta data in compressed TX
  * ANDROID: GKI: ALSA: compress: Add DSD format support for ALSA
  * ANDROID: GKI: ASoC: msm: qdsp6v2: add support for ALAC and APE offload
  * ANDROID: GKI: SoC: msm: Add compressed TX and passthrough support
  * ANDROID: GKI: ASoC: msm: qdsp6v2: Add FLAC in compress offload path
  * ANDROID: GKI: ASoC: msm: add support for different compressed formats
  * ANDROID: GKI: ASoC: msm: Update the encode option and sample rate
  * ANDROID: GKI: Enable CONFIG_SND_VERBOSE_PROCFS in gki_defconfig
  * ANDROID: GKI: Add hidden CONFIG_SND_SOC_COMPRESS to gki_defconfig
  * ANDROID: GKI: ALSA: pcm: add locks for accessing runtime resource
  * ANDROID: GKI: Update ABI for DRM changes
  * ANDROID: GKI: Add drm_dp_send_dpcd_{read,write} accessor functions
  * ANDROID: GKI: drm: Add drm_dp_mst_get_max_sdp_streams_supported accessor function
  * ANDROID: GKI: drm: Add drm_dp_mst_has_fec accessor function
  * ANDROID: GKI: Add 'dsc_info' to struct drm_dp_mst_port
  * ANDROID: GKI: usb: Add support to handle USB SMMU S1 address
  * ANDROID: GKI: usb: Add helper APIs to return xhci phys addresses
  * ANDROID: Add C protos for dma_buf/drm_prime get_uuid
  * ANDROID: GKI: Make linux_banner a C pointer
  * ANDROID: GKI: Add 'refresh_rate', 'id' to struct drm_panel_notifier
  * ANDROID: GKI: Add 'i2c_mutex' to struct drm_dp_aux
  * ANDROID: GKI: Add 'checksum' to struct drm_connector
  * Revert "BACKPORT: drm: Add HDR source metadata property"
  * Revert "BACKPORT: drm: Parse HDR metadata info from EDID"
  * ANDROID: drm: Add DP colorspace property
  * ANDROID: GKI: drm: Initialize display->hdmi when parsing vsdb
  * ANDROID: drivers: gpu: drm: add support to batch commands
  * ANDROID: ABI: update the qcom whitelist
  * ANDROID: GKI: ARM64: smp: add vendor field pending_ipi
  * ANDROID: gki_defconfig: enable msm serial early console
  * ANDROID: serial: msm_geni_serial_console : Add Earlycon support
  * ANDROID: GKI: serial: core: export uart_console_device
  * f2fs: fix quota_sync failure due to f2fs_lock_op
  * f2fs: support read iostat
  * f2fs: Fix the accounting of dcc->undiscard_blks
  * f2fs: fix to handle error path of f2fs_ra_meta_pages()
  * f2fs: report the discard cmd errors properly
  * f2fs: fix long latency due to discard during umount
  * f2fs: add tracepoint for f2fs iostat
  * f2fs: introduce sysfs/data_io_flag to attach REQ_META/FUA
  * ANDROID: GKI: update abi definition due to previous changes in the tree
  * Linux 4.19.116
  * efi/x86: Fix the deletion of variables in mixed mode
  * mfd: dln2: Fix sanity checking for endpoints
  * etnaviv: perfmon: fix total and idle HI cyleces readout
  * misc: echo: Remove unnecessary parentheses and simplify check for zero
  * powerpc/fsl_booke: Avoid creating duplicate tlb1 entry
  * ftrace/kprobe: Show the maxactive number on kprobe_events
  * drm: Remove PageReserved manipulation from drm_pci_alloc
  * drm/dp_mst: Fix clearing payload state on topology disable
  * Revert "drm/dp_mst: Remove VCPI while disabling topology mgr"
  * crypto: ccree - only try to map auth tag if needed
  * crypto: ccree - dec auth tag size from cryptlen map
  * crypto: ccree - don't mangle the request assoclen
  * crypto: ccree - zero out internal struct before use
  * crypto: ccree - improve error handling
  * crypto: caam - update xts sector size for large input length
  * dm zoned: remove duplicate nr_rnd_zones increase in dmz_init_zone()
  * btrfs: use nofs allocations for running delayed items
  * powerpc: Make setjmp/longjmp signature standard
  * powerpc: Add attributes for setjmp/longjmp
  * scsi: mpt3sas: Fix kernel panic observed on soft HBA unplug
  * powerpc/kprobes: Ignore traps that happened in real mode
  * powerpc/xive: Use XIVE_BAD_IRQ instead of zero to catch non configured IPIs
  * powerpc/hash64/devmap: Use H_PAGE_THP_HUGE when setting up huge devmap PTE entries
  * powerpc/64/tm: Don't let userspace set regs->trap via sigreturn
  * powerpc/powernv/idle: Restore AMR/UAMOR/AMOR after idle
  * xen/blkfront: fix memory allocation flags in blkfront_setup_indirect()
  * ipmi: fix hung processes in __get_guid()
  * libata: Return correct status in sata_pmp_eh_recover_pm() when ATA_DFLAG_DETACH is set
  * hfsplus: fix crash and filesystem corruption when deleting files
  * cpufreq: powernv: Fix use-after-free
  * kmod: make request_module() return an error when autoloading is disabled
  * clk: ingenic/jz4770: Exit with error if CGU init failed
  * Input: i8042 - add Acer Aspire 5738z to nomux list
  * s390/diag: fix display of diagnose call statistics
  * perf tools: Support Python 3.8+ in Makefile
  * ocfs2: no need try to truncate file beyond i_size
  * fs/filesystems.c: downgrade user-reachable WARN_ONCE() to pr_warn_once()
  * ext4: fix a data race at inode->i_blocks
  * NFS: Fix a page leak in nfs_destroy_unlinked_subrequests()
  * powerpc/pseries: Avoid NULL pointer dereference when drmem is unavailable
  * drm/etnaviv: rework perfmon query infrastructure
  * rtc: omap: Use define directive for PIN_CONFIG_ACTIVE_HIGH
  * selftests: vm: drop dependencies on page flags from mlock2 tests
  * arm64: armv8_deprecated: Fix undef_hook mask for thumb setend
  * scsi: zfcp: fix missing erp_lock in port recovery trigger for point-to-point
  * dm verity fec: fix memory leak in verity_fec_dtr
  * dm writecache: add cond_resched to avoid CPU hangs
  * arm64: dts: allwinner: h6: Fix PMU compatible
  * net: qualcomm: rmnet: Allow configuration updates to existing devices
  * mm: Use fixed constant in page_frag_alloc instead of size + 1
  * tools: gpio: Fix out-of-tree build regression
  * x86/speculation: Remove redundant arch_smt_update() invocation
  * powerpc/pseries: Drop pointless static qualifier in vpa_debugfs_init()
  * erofs: correct the remaining shrink objects
  * crypto: mxs-dcp - fix scatterlist linearization for hash
  * btrfs: fix missing semaphore unlock in btrfs_sync_file
  * btrfs: fix missing file extent item for hole after ranged fsync
  * btrfs: drop block from cache on error in relocation
  * btrfs: set update the uuid generation as soon as possible
  * Btrfs: fix crash during unmount due to race with delayed inode workers
  * mtd: spinand: Do not erase the block before writing a bad block marker
  * mtd: spinand: Stop using spinand->oobbuf for buffering bad block markers
  * CIFS: Fix bug which the return value by asynchronous read is error
  * KVM: VMX: fix crash cleanup when KVM wasn't used
  * KVM: x86: Gracefully handle __vmalloc() failure during VM allocation
  * KVM: VMX: Always VMCLEAR in-use VMCSes during crash with kexec support
  * KVM: x86: Allocate new rmap and large page tracking when moving memslot
  * KVM: s390: vsie: Fix delivery of addressing exceptions
  * KVM: s390: vsie: Fix region 1 ASCE sanity shadow address checks
  * KVM: nVMX: Properly handle userspace interrupt window request
  * x86/entry/32: Add missing ASM_CLAC to general_protection entry
  * signal: Extend exec_id to 64bits
  * ath9k: Handle txpower changes even when TPC is disabled
  * MIPS: OCTEON: irq: Fix potential NULL pointer dereference
  * MIPS/tlbex: Fix LDDIR usage in setup_pw() for Loongson-3
  * pstore: pstore_ftrace_seq_next should increase position index
  * irqchip/versatile-fpga: Apply clear-mask earlier
  * KEYS: reaching the keys quotas correctly
  * tpm: tpm2_bios_measurements_next should increase position index
  * tpm: tpm1_bios_measurements_next should increase position index
  * tpm: Don't make log failures fatal
  * PCI: endpoint: Fix for concurrent memory allocation in OB address region
  * PCI: Add boot interrupt quirk mechanism for Xeon chipsets
  * PCI/ASPM: Clear the correct bits when enabling L1 substates
  * PCI: pciehp: Fix indefinite wait on sysfs requests
  * nvme: Treat discovery subsystems as unique subsystems
  * nvme-fc: Revert "add module to ops template to allow module references"
  * thermal: devfreq_cooling: inline all stubs for CONFIG_DEVFREQ_THERMAL=n
  * acpi/x86: ignore unspecified bit positions in the ACPI global lock field
  * media: ti-vpe: cal: fix disable_irqs to only the intended target
  * ALSA: hda/realtek - Add quirk for MSI GL63
  * ALSA: hda/realtek - Remove now-unnecessary XPS 13 headphone noise fixups
  * ALSA: hda/realtek - Set principled PC Beep configuration for ALC256
  * ALSA: doc: Document PC Beep Hidden Register on Realtek ALC256
  * ALSA: pcm: oss: Fix regression by buffer overflow fix
  * ALSA: ice1724: Fix invalid access for enumerated ctl items
  * ALSA: hda: Fix potential access overflow in beep helper
  * ALSA: hda: Add driver blacklist
  * ALSA: usb-audio: Add mixer workaround for TRX40 and co
  * usb: gadget: composite: Inform controller driver of self-powered
  * usb: gadget: f_fs: Fix use after free issue as part of queue failure
  * ASoC: topology: use name_prefix for new kcontrol
  * ASoC: dpcm: allow start or stop during pause for backend
  * ASoC: dapm: connect virtual mux with default value
  * ASoC: fix regwmask
  * slub: improve bit diffusion for freelist ptr obfuscation
  * uapi: rename ext2_swab() to swab() and share globally in swab.h
  * IB/mlx5: Replace tunnel mpls capability bits for tunnel_offloads
  * btrfs: track reloc roots based on their commit root bytenr
  * btrfs: remove a BUG_ON() from merge_reloc_roots()
  * btrfs: qgroup: ensure qgroup_rescan_running is only set when the worker is at least queued
  * block, bfq: fix use-after-free in bfq_idle_slice_timer_body
  * locking/lockdep: Avoid recursion in lockdep_count_{for,back}ward_deps()
  * firmware: fix a double abort case with fw_load_sysfs_fallback
  * md: check arrays is suspended in mddev_detach before call quiesce operations
  * irqchip/gic-v4: Provide irq_retrigger to avoid circular locking dependency
  * usb: dwc3: core: add support for disabling SS instances in park mode
  * media: i2c: ov5695: Fix power on and off sequences
  * block: Fix use-after-free issue accessing struct io_cq
  * genirq/irqdomain: Check pointer in irq_domain_alloc_irqs_hierarchy()
  * efi/x86: Ignore the memory attributes table on i386
  * x86/boot: Use unsigned comparison for addresses
  * gfs2: Don't demote a glock until its revokes are written
  * pstore/platform: fix potential mem leak if pstore_init_fs failed
  * libata: Remove extra scsi_host_put() in ata_scsi_add_hosts()
  * media: i2c: video-i2c: fix build errors due to 'imply hwmon'
  * PCI/switchtec: Fix init_completion race condition with poll_wait()
  * selftests/x86/ptrace_syscall_32: Fix no-vDSO segfault
  * sched: Avoid scale real weight down to zero
  * irqchip/versatile-fpga: Handle chained IRQs properly
  * block: keep bdi->io_pages in sync with max_sectors_kb for stacked devices
  * x86: Don't let pgprot_modify() change the page encryption bit
  * xhci: bail out early if driver can't accress host in resume
  * null_blk: fix spurious IO errors after failed past-wp access
  * null_blk: Handle null_add_dev() failures properly
  * null_blk: Fix the null_add_dev() error path
  * firmware: arm_sdei: fix double-lock on hibernate with shared events
  * media: venus: hfi_parser: Ignore HEVC encoding for V1
  * cpufreq: imx6q: Fixes unwanted cpu overclocking on i.MX6ULL
  * i2c: st: fix missing struct parameter description
  * qlcnic: Fix bad kzalloc null test
  * cxgb4/ptp: pass the sign of offset delta in FW CMD
  * hinic: fix wrong para of wait_for_completion_timeout
  * hinic: fix a bug of waitting for IO stopped
  * net: vxge: fix wrong __VA_ARGS__ usage
  * bus: sunxi-rsb: Return correct data when mixing 16-bit and 8-bit reads
  * ARM: dts: sun8i-a83t-tbs-a711: HM5065 doesn't like such a high voltage
  * ANDROID: build.config.allmodconfig: Re-enable XFS_FS
  * FROMGIT: of: property: Add device link support for extcon
  * ANDROID: GKI: arm64: gki_defconfig: enable CONFIG_MM_EVENT_STAT
  * ANDROID: GKI: add fields from per-process mm event tracking feature
  * ANDROID: GKI: fix ABI diffs caused by ION heap and pool vmstat additions
  * UPSTREAM: GKI: panic/reboot: allow specifying reboot_mode for panic only
  * ANDROID: GKI: of: property: Add device link support for phys property
  * ANDROID: GKI: usb: phy: Fix ABI diff for usb_otg_state
  * ANDROID: GKI: usb: phy: Fix ABI diff due to usb_phy.drive_dp_pulse
  * ANDROID: GKI: usb: phy: Fix ABI diff for usb_phy_type and usb_phy.reset
  * ANDROID: gki_defconfig: enable CONFIG_GPIO_SYSFS
  * ANDROID: GKI: qcom: Fix compile issue when setting msm_lmh_dcvs as a module
  * ANDROID: GKI: drivers: cpu_cooling: allow platform freq mitigation
  * ANDROID: GKI: ASoC: Add locking in DAPM widget power update
  * ANDROID: GKI: ASoC: jack: Fix buttons enum value
  * ANDROID: GKI: ALSA: jack: Add support to report second microphone
  * ANDROID: GKI: ALSA: jack: Update supported jack switch types
  * ANDROID: GKI: ALSA: jack: update jack types
  * ANDROID: GKI: Export symbols arm_cpuidle_suspend, cpuidle_dev and cpuidle_register_governor
  * ANDROID: GKI: usb: hcd: Add USB atomic notifier callback for HC died error
  * ANDROID: media: increase video max frame number
  * BACKPORT: nvmem: core: add NVMEM_SYSFS Kconfig
  * UPSTREAM: nvmem: add support for cell info
  * UPSTREAM: nvmem: remove the global cell list
  * UPSTREAM: nvmem: use kref
  * UPSTREAM: nvmem: use list_for_each_entry_safe in nvmem_device_remove_all_cells()
  * UPSTREAM: nvmem: provide nvmem_dev_name()
  * ANDROID: GKI: Bulk ABI update
  * ANDROID: GKI: cpuhotplug: adding hotplug enums for vendor code
  * ANDROID: Incremental fs: Fix create_file performance
  * ANDROID: build.config.common: Add BUILDTOOLS_PREBUILT_BIN
  * UPSTREAM: kheaders: include only headers into kheaders_data.tar.xz
  * UPSTREAM: kheaders: remove meaningless -R option of 'ls'
  * ANDROID: GKI: of: platform: initialize of_reserved_mem
  * ANDROID: driver: gpu: drm: add notifier for panel related events
  * ANDROID: include: drm: support unicasting mipi cmds to dsi ctrls
  * ANDROID: include: drm: increase DRM max property count to 64
  * BACKPORT: drm: Add HDMI colorspace property
  * ANDROID: drm: edid: add support for additional CEA extension blocks
  * BACKPORT: drm: Parse HDR metadata info from EDID
  * BACKPORT: drm: Add HDR source metadata property
  * BACKPORT: drm/dp_mst: Parse FEC capability on MST ports
  * ANDROID: GKI: ABI update for DRM changes
  * ANDROID: ABI: add missing elf variables to representation
  * ANDROID: GKI: power_supply: Add PROP_MOISTURE_DETECTION_ENABLED
  * ANDROID: include: drm: add the definitions for DP Link Compliance tests
  * ANDROID: drivers: gpu: drm: fix bugs encountered while fuzzing
  * FROMLIST: power_supply: Add additional health properties to the header
  * UPSTREAM: power: supply: core: Update sysfs-class-power ABI document
  * UPSTREAM: Merge remote-tracking branch 'aosp/upstream-f2fs-stable-linux-4.19.y' into android-4.19 (v5.7-rc1)
  * ANDROID: drivers: gpu: drm: add support for secure framebuffer
  * ANDROID: include: uapi: drm: add additional QCOM modifiers
  * ANDROID: drm: dsi: add two DSI mode flags for BLLP
  * ANDROID: include: uapi: drm: add additional drm mode flags
  * UPSTREAM: drm: plug memory leak on drm_setup() failure
  * UPSTREAM: drm: factor out drm_close_helper() function
  * ANDROID: GKI: Bulk ABI update
  * BACKPORT: nl80211: Add per peer statistics to compute FCS error rate
  * ANDROID: GKI: sound: usb: Add snd_usb_enable_audio_stream/find_snd_usb_substream
  * ANDROID: GKI: add dma-buf includes
  * ANDROID: GKI: sched: struct fields for Per-Sched-domain over utilization
  * ANDROID: GKI: Add vendor fields to root_domain
  * ANDROID: gki_defconfig: Enable CONFIG_IRQ_TIME_ACCOUNTING
  * ANDROID: fix allmodconfig build to use the right toolchain
  * ANDROID: fix allmodconfig build to use the right toolchain
  * ANDROID: GKI: Update ABI
  * Revert "UPSTREAM: mm, page_alloc: spread allocations across zones before introducing fragmentation"
  * Revert "UPSTREAM: mm: use alloc_flags to record if kswapd can wake"
  * Revert "BACKPORT: mm: move zone watermark accesses behind an accessor"
  * Revert "BACKPORT: mm: reclaim small amounts of memory when an external fragmentation event occurs"
  * Revert "BACKPORT: mm, compaction: be selective about what pageblocks to clear skip hints"
  * ANDROID: GKI: panic: add vendor callback function in panic()
  * UPSTREAM: GKI: thermal: make device_register's type argument const
  * ANDROID: GKI: add base.h include to match MODULE_VERSIONS
  * ANDROID: update the ABI based on the new whitelist
  * ANDROID: GKI: fdt: export symbols required by modules
  * ANDROID: GKI: drivers: of: Add APIs to find DDR device rank, HBB
  * ANDROID: GKI: security: Add mmap export symbols for modules
  * ANDROID: GKI: arch: add stub symbols for boot_reason and cold_boot
  * ANDROID: GKI: USB: Fix ABI diff for struct usb_bus
  * ANDROID: GKI: USB: Resolve ABI diff for usb_gadget and usb_gadget_ops
  * ANDROID: GKI: add hidden V4L2_MEM2MEM_DEV
  * ANDROID: GKI: enable VIDEO_V4L2_SUBDEV_API
  * ANDROID: GKI: export symbols from abi_gki_aarch64_qcom_whitelist
  * ANDROID: Update the whitelist for qcom SoCs
  * ANDROID: Incremental fs: Fix compound page usercopy crash
  * ANDROID: Incremental fs: Clean up incfs_test build process
  * ANDROID: Incremental fs: make remount log buffer change atomic
  * ANDROID: Incremental fs: Optimize get_filled_block
  * ANDROID: Incremental fs: Fix mislabeled __user ptrs
  * ANDROID: Incremental fs: Use 64-bit int for file_size when writing hash blocks
  * Merge remote-tracking branch 'aosp/upstream-f2fs-stable-linux-4.19.y' into android-4.19 (v5.7-rc1)
  * Merge 4.19.115 into android-4.19
  * Linux 4.19.115
  * drm/msm: Use the correct dma_sync calls in msm_gem
  * drm_dp_mst_topology: fix broken drm_dp_sideband_parse_remote_dpcd_read()
  * usb: dwc3: don't set gadget->is_otg flag
  * rpmsg: glink: Remove chunk size word align warning
  * arm64: Fix size of __early_cpu_boot_status
  * drm/msm: stop abusing dma_map/unmap for cache
  * clk: qcom: rcg: Return failure for RCG update
  * fbcon: fix null-ptr-deref in fbcon_switch
  * RDMA/cm: Update num_paths in cma_resolve_iboe_route error flow
  * Bluetooth: RFCOMM: fix ODEBUG bug in rfcomm_dev_ioctl
  * RDMA/cma: Teach lockdep about the order of rtnl and lock
  * RDMA/ucma: Put a lock around every call to the rdma_cm layer
  * ceph: canonicalize server path in place
  * ceph: remove the extra slashes in the server path
  * IB/hfi1: Fix memory leaks in sysfs registration and unregistration
  * IB/hfi1: Call kobject_put() when kobject_init_and_add() fails
  * ASoC: jz4740-i2s: Fix divider written at incorrect offset in register
  * hwrng: imx-rngc - fix an error path
  * tools/accounting/getdelays.c: fix netlink attribute length
  * usb: dwc3: gadget: Wrap around when skip TRBs
  * random: always use batched entropy for get_random_u{32,64}
  * mlxsw: spectrum_flower: Do not stop at FLOW_ACTION_VLAN_MANGLE
  * slcan: Don't transmit uninitialized stack data in padding
  * net: stmmac: dwmac1000: fix out-of-bounds mac address reg setting
  * net: phy: micrel: kszphy_resume(): add delay after genphy_resume() before accessing PHY registers
  * net: dsa: bcm_sf2: Ensure correct sub-node is parsed
  * net: dsa: bcm_sf2: Do not register slave MDIO bus with OF
  * ipv6: don't auto-add link-local address to lag ports
  * mm: mempolicy: require at least one nodeid for MPOL_PREFERRED
  * include/linux/notifier.h: SRCU: fix ctags
  * bitops: protect variables in set_mask_bits() macro
  * padata: always acquire cpu_hotplug_lock before pinst->lock
  * net: Fix Tx hash bound checking
  * rxrpc: Fix sendmsg(MSG_WAITALL) handling
  * ALSA: hda/ca0132 - Add Recon3Di quirk to handle integrated sound on EVGA X99 Classified motherboard
  * power: supply: axp288_charger: Add special handling for HP Pavilion x2 10
  * extcon: axp288: Add wakeup support
  * mei: me: add cedar fork device ids
  * coresight: do not use the BIT() macro in the UAPI header
  * misc: pci_endpoint_test: Avoid using module parameter to determine irqtype
  * misc: pci_endpoint_test: Fix to support > 10 pci-endpoint-test devices
  * misc: rtsx: set correct pcr_ops for rts522A
  * media: rc: IR signal for Panasonic air conditioner too long
  * drm/etnaviv: replace MMU flush marker with flush sequence
  * tools/power turbostat: Fix missing SYS_LPI counter on some Chromebooks
  * tools/power turbostat: Fix gcc build warnings
  * drm/amdgpu: fix typo for vcn1 idle check
  * initramfs: restore default compression behavior
  * drm/bochs: downgrade pci_request_region failure from error to warning
  * drm/amd/display: Add link_rate quirk for Apple 15" MBP 2017
  * nvme-rdma: Avoid double freeing of async event data
  * sctp: fix possibly using a bad saddr with a given dst
  * sctp: fix refcount bug in sctp_wfree
  * net, ip_tunnel: fix interface lookup with no key
  * ipv4: fix a RCU-list lock in fib_triestat_seq_show
  * Merge remote-tracking branch 'aosp/android-4.19' into android-4.19-stable
  * ANDROID: GKI: export symbols required by SPECTRA_CAMERA
  * ANDROID: GKI: ARM/ARM64: Introduce arch_read_hardware_id
  * ANDROID: GKI: drivers: base: soc: export symbols for socinfo
  * ANDROID: GKI: Update ABI
  * ANDROID: GKI: ASoC: msm: fix integer overflow for long duration offload playback
  * ANDROID: GKI: Bulk ABI update
  * Revert "ANDROID: GKI: mm: add struct/enum fields for SPECULATIVE_PAGE_FAULTS"
  * ANDROID: GKI: Revert "arm64: kill flush_cache_all()"
  * ANDROID: GKI: Revert "arm64: Remove unused macros from assembler.h"
  * ANDROID: GKI: kernel/dma, mm/cma: Export symbols needed by vendor modules
  * ANDROID: GKI: mm: Export symbols __next_zones_zonelist and zone_watermark_ok_safe
  * ANDROID: GKI: mm/memblock: export memblock_overlaps_memory
  * ANDROID: GKI: net, skbuff: export symbols needed by vendor drivers
  * ANDROID: GKI: Add stub __cpu_isolated_mask symbol
  * ANDROID: GKI: sched: stub sched_isolate symbols
  * ANDROID: GKI: export saved_command_line
  * ANDROID: GKI: Update ABI
  * ANDROID: GKI: ASoC: core: Update ALSA core to issue restart in underrun.
  * ANDROID: GKI: SoC: pcm: Add a restart callback field to struct snd_pcm_ops
  * ANDROID: GKI: SoC: pcm: Add fields to struct snd_pcm_ops and struct snd_soc_component_driver
  * ANDROID: GKI: ASoC: core: Add compat_ioctl callback to struct snd_pcm_ops
  * ANDROID: GKI: ALSA: core: modify, rename and export create_subdir API
  * ANDROID: GKI: usb: Add helper API to issue stop endpoint command
  * ANDROID: GKI: Thermal: thermal_zone_get_cdev_by_name added
  * ANDROID: GKI: add missing exports for CONFIG_ARM_SMMU=m
  * ANDROID: power: wakeup_reason: wake reason enhancements
  * BACKPORT: FROMGIT: kbuild: mkcompile_h: Include $LD version in /proc/version
  * ANDROID: GKI: kernel: Export symbols needed by msm_minidump.ko and minidump_log.ko
  * ubifs: wire up FS_IOC_GET_ENCRYPTION_NONCE
  * f2fs: wire up FS_IOC_GET_ENCRYPTION_NONCE
  * ext4: wire up FS_IOC_GET_ENCRYPTION_NONCE
  * fscrypt: add FS_IOC_GET_ENCRYPTION_NONCE ioctl
  * ANDROID: Bulk update the ABI xml
  * ANDROID: gki_defconfig: add CONFIG_IPV6_SUBTREES
  * ANDROID: GKI: arm64: reserve space in cpu_hwcaps and cpu_hwcap_keys arrays
  * ANDROID: GKI: of: reserved_mem: Fix kmemleak crash on no-map region
  * ANDROID: GKI: sched: add task boost vendor fields to task_struct
  * ANDROID: GKI: mm: add rss counter for unreclaimable pages
  * ANDROID: GKI: irqdomain: add bus token DOMAIN_BUS_WAKEUP
  * ANDROID: GKI: arm64: fault: do_tlb_conf_fault_cb register fault callback
  * ANDROID: GKI: QoS: Enhance framework to support cpu/irq specific QoS requests
  * ANDROID: GKI: Bulk ABI update
  * ANDROID: GKI: PM/devfreq: Do not switch governors from sysfs when device is suspended
  * ANDROID: GKI: PM / devfreq: Fix race condition between suspend/resume and governor_store
  * ANDROID: GKI: PM / devfreq: Introduce a sysfs lock
  * ANDROID: GKI: regmap: irq: Add support to clear ack registers
  * ANDROID: GKI: Remove SCHED_AUTOGROUP
  * ANDROID: ignore compiler tag __must_check for GENKSYMS
  * ANDROID: GKI: Bulk update ABI
  * ANDROID: GKI: Fix ABI diff for struct thermal_cooling_device_ops
  * ANDROID: GKI: ASoC: soc-core: export function to find components
  * ANDROID: GKI: thermal: thermal_sys: Add configurable thermal trip points.
  * ANDROID: fscrypt: fall back to filesystem-layer crypto when needed
  * ANDROID: block: require drivers to declare supported crypto key type(s)
  * ANDROID: block: make blk_crypto_start_using_mode() properly check for support
  * ANDROID: GKI: power: supply: format regression
  * ANDROID: GKI: kobject: increase number of kobject uevent pointers to 64
  * ANDROID: GKI: drivers: video: backlight: Fix ABI diff for struct backlight_device
  * ANDROID: GKI: usb: xhci: Add support for secondary interrupters
  * ANDROID: GKI: usb: host: xhci: Add support for usb core indexing
  * ANDROID: gki_defconfig: enable USB_XHCI_HCD
  * ANDROID: gki_defconfig: enable CONFIG_BRIDGE
  * ANDROID: GKI: Update ABI report
  * ANDROID: GKI: arm64: smp: Add set_update_ipi_history_callback
  * ANDROID: kbuild: ensure __cfi_check is correctly aligned
  * f2fs: keep inline_data when compression conversion
  * f2fs: fix to disable compression on directory
  * f2fs: add missing CONFIG_F2FS_FS_COMPRESSION
  * f2fs: switch discard_policy.timeout to bool type
  * f2fs: fix to verify tpage before releasing in f2fs_free_dic()
  * f2fs: show compression in statx
  * f2fs: clean up dic->tpages assignment
  * f2fs: compress: support zstd compress algorithm
  * f2fs: compress: add .{init,destroy}_decompress_ctx callback
  * f2fs: compress: fix to call missing destroy_compress_ctx()
  * f2fs: change default compression algorithm
  * f2fs: clean up {cic,dic}.ref handling
  * f2fs: fix to use f2fs_readpage_limit() in f2fs_read_multi_pages()
  * f2fs: xattr.h: Make stub helpers inline
  * f2fs: fix to avoid double unlock
  * f2fs: fix potential .flags overflow on 32bit architecture
  * f2fs: fix NULL pointer dereference in f2fs_verity_work()
  * f2fs: fix to clear PG_error if fsverity failed
  * f2fs: don't call fscrypt_get_encryption_info() explicitly in f2fs_tmpfile()
  * f2fs: don't trigger data flush in foreground operation
  * f2fs: fix NULL pointer dereference in f2fs_write_begin()
  * f2fs: clean up f2fs_may_encrypt()
  * f2fs: fix to avoid potential deadlock
  * f2fs: don't change inode status under page lock
  * f2fs: fix potential deadlock on compressed quota file
  * f2fs: delete DIO read lock
  * f2fs: don't mark compressed inode dirty during f2fs_iget()
  * f2fs: fix to account compressed blocks in f2fs_compressed_blocks()
  * f2fs: xattr.h: Replace zero-length array with flexible-array member
  * f2fs: fix to update f2fs_super_block fields under sb_lock
  * f2fs: Add a new CP flag to help fsck fix resize SPO issues
  * f2fs: Fix mount failure due to SPO after a successful online resize FS
  * f2fs: use kmem_cache pool during inline xattr lookups
  * f2fs: skip migration only when BG_GC is called
  * f2fs: fix to show tracepoint correctly
  * f2fs: avoid __GFP_NOFAIL in f2fs_bio_alloc
  * f2fs: introduce F2FS_IOC_GET_COMPRESS_BLOCKS
  * f2fs: fix to avoid triggering IO in write path
  * f2fs: add prefix for f2fs slab cache name
  * f2fs: introduce DEFAULT_IO_TIMEOUT
  * f2fs: skip GC when section is full
  * f2fs: add migration count iff migration happens
  * f2fs: clean up bggc mount option
  * f2fs: clean up lfs/adaptive mount option
  * f2fs: fix to show norecovery mount option
  * f2fs: clean up parameter of macro XATTR_SIZE()
  * f2fs: clean up codes with {f2fs_,}data_blkaddr()
  * f2fs: show mounted time
  * f2fs: Use scnprintf() for avoiding potential buffer overflow
  * f2fs: allow to clear F2FS_COMPR_FL flag
  * f2fs: fix to check dirty pages during compressed inode conversion
  * f2fs: fix to account compressed inode correctly
  * f2fs: fix wrong check on F2FS_IOC_FSSETXATTR
  * f2fs: fix to avoid use-after-free in f2fs_write_multi_pages()
  * f2fs: fix to avoid using uninitialized variable
  * f2fs: fix inconsistent comments
  * f2fs: remove i_sem lock coverage in f2fs_setxattr()
  * f2fs: cover last_disk_size update with spinlock
  * f2fs: fix to check i_compr_blocks correctly
  * FROMLIST: kmod: make request_module() return an error when autoloading is disabled
  * ANDROID: GKI: Update ABI report
  * ANDROID: GKI: ARM64: dma-mapping: export symbol arch_setup_dma_ops
  * ANDROID: GKI: ARM: dma-mapping: export symbol arch_setup_dma_ops
  * ANDROID: GKI: ASoC: dapm: Avoid static route b/w cpu and codec dai
  * ANDROID: GKI: ASoC: pcm: Add support for hostless playback/capture
  * ANDROID: GKI: ASoC: core - add hostless DAI support
  * ANDROID: GKI: drivers: thermal: Resolve ABI diff for struct thermal_zone_device_ops
  * ANDROID: GKI: drivers: thermal: Add support for getting trip temperature
  * ANDROID: GKI: Add functions of_thermal_handle_trip/of_thermal_handle_trip_temp
  * ANDROID: GKI: drivers: thermal: Add post suspend evaluate flag to thermal zone devicetree
  * UPSTREAM: loop: Only freeze block queue when needed.
  * UPSTREAM: loop: Only change blocksize when needed.
  * ANDROID: Fix wq fp check for CFI builds
  * ANDROID: GKI: update abi definition after CONFIG_DEBUG_LIST was enabled
  * ANDROID: gki_defconfig: enable CONFIG_DEBUG_LIST
  * ANDROID: GKI: Update ABI definition
  * ANDROID: GKI: remove condition causing sk_buff struct ABI differences
  * ANDROID: GKI: Export symbol arch_timer_mem_get_cval
  * ANDROID: GKI: pwm: core: Add option to config PWM duty/period with u64 data length
  * ANDROID: Update ABI whitelist for qcom SoCs
  * ANDROID: Incremental fs: Fix remount
  * ANDROID: Incremental fs: Protect get_fill_block, and add a field
  * ANDROID: Incremental fs: Fix crash polling 0 size read_log
  * ANDROID: Incremental fs: get_filled_blocks: better index_out
  * ANDROID: GKI: of: property: Add device links support for "qcom,wrapper-dev"
  * ANDROID: GKI: update abi definitions due to recent changes
  * Merge 4.19.114 into android-4.19
  * ANDROID: GKI: clk: Initialize in stack clk_init_data to 0 in all drivers
  * ANDROID: GKI: drivers: clksource: Add API to return cval
  * ANDROID: GKI: clk: Add support for voltage voting
  * ANDROID: GKI: kernel: Export task and IRQ affinity symbols
  * ANDROID: GKI: regulator: core: Add support for regulator providers with sync state
  * ANDROID: GKI: regulator: Call proxy-consumer functions for each regulator registered
  * ANDROID: GKI: regulator: Add proxy consumer driver
  * ANDROID: GKI: regulator: core: allow long device tree supply regulator property names
  * ANDROID: GKI: Revert "regulator: Enable supply regulator if child rail is enabled."
  * ANDROID: GKI: regulator: Remove redundant set_mode call in drms_uA_update
  * ANDROID: GKI: net: Add the get current NAPI context API
  * ANDROID: GKI: remove DRM_KMS_CMA_HELPER from GKI configuration
  * ANDROID: GKI: edac: Fix ABI diffs in edac_device_ctl_info struct
  * ANDROID: GKI: pwm: Add different PWM output types support
  * UPSTREAM: cfg80211: Authentication offload to user space in AP mode
  * Linux 4.19.114
  * arm64: dts: ls1046ardb: set RGMII interfaces to RGMII_ID mode
  * arm64: dts: ls1043a-rdb: correct RGMII delay mode to rgmii-id
  * ARM: dts: N900: fix onenand timings
  * ARM: dts: imx6: phycore-som: fix arm and soc minimum voltage
  * ARM: bcm2835-rpi-zero-w: Add missing pinctrl name
  * ARM: dts: oxnas: Fix clear-mask property
  * perf map: Fix off by one in strncpy() size argument
  * arm64: alternative: fix build with clang integrated assembler
  * net: ks8851-ml: Fix IO operations, again
  * gpiolib: acpi: Add quirk to ignore EC wakeups on HP x2 10 CHT + AXP288 model
  * bpf: Explicitly memset some bpf info structures declared on the stack
  * bpf: Explicitly memset the bpf_attr structure
  * platform/x86: pmc_atom: Add Lex 2I385SW to critclk_systems DMI table
  * vt: vt_ioctl: fix use-after-free in vt_in_use()
  * vt: vt_ioctl: fix VT_DISALLOCATE freeing in-use virtual console
  * vt: vt_ioctl: remove unnecessary console allocation checks
  * vt: switch vt_dont_switch to bool
  * vt: ioctl, switch VT_IS_IN_USE and VT_BUSY to inlines
  * vt: selection, introduce vc_is_sel
  * mac80211: fix authentication with iwlwifi/mvm
  * mac80211: Check port authorization in the ieee80211_tx_dequeue() case
  * media: xirlink_cit: add missing descriptor sanity checks
  * media: stv06xx: add missing descriptor sanity checks
  * media: dib0700: fix rc endpoint lookup
  * media: ov519: add missing endpoint sanity checks
  * libfs: fix infoleak in simple_attr_read()
  * ahci: Add Intel Comet Lake H RAID PCI ID
  * staging: wlan-ng: fix use-after-free Read in hfa384x_usbin_callback
  * staging: wlan-ng: fix ODEBUG bug in prism2sta_disconnect_usb
  * staging: rtl8188eu: Add ASUS USB-N10 Nano B1 to device table
  * media: usbtv: fix control-message timeouts
  * media: flexcop-usb: fix endpoint sanity check
  * usb: musb: fix crash with highmen PIO and usbmon
  * USB: serial: io_edgeport: fix slab-out-of-bounds read in edge_interrupt_callback
  * USB: cdc-acm: restore capability check order
  * USB: serial: option: add Wistron Neweb D19Q1
  * USB: serial: option: add BroadMobi BM806U
  * USB: serial: option: add support for ASKEY WWHC050
  * mac80211: set IEEE80211_TX_CTRL_PORT_CTRL_PROTO for nl80211 TX
  * mac80211: add option for setting control flags
  * Revert "r8169: check that Realtek PHY driver module is loaded"
  * vti6: Fix memory leak of skb if input policy check fails
  * bpf/btf: Fix BTF verification of enum members in struct/union
  * netfilter: nft_fwd_netdev: validate family and chain type
  * netfilter: flowtable: reload ip{v6}h in nf_flow_tuple_ip{v6}
  * afs: Fix some tracing details
  * xfrm: policy: Fix doulbe free in xfrm_policy_timer
  * xfrm: add the missing verify_sec_ctx_len check in xfrm_add_acquire
  * xfrm: fix uctx len check in verify_sec_ctx_len
  * RDMA/mlx5: Block delay drop to unprivileged users
  * vti[6]: fix packet tx through bpf_redirect() in XinY cases
  * xfrm: handle NETDEV_UNREGISTER for xfrm device
  * genirq: Fix reference leaks on irq affinity notifiers
  * RDMA/core: Ensure security pkey modify is not lost
  * gpiolib: acpi: Add quirk to ignore EC wakeups on HP x2 10 BYT + AXP288 model
  * gpiolib: acpi: Rework honor_wakeup option into an ignore_wake option
  * gpiolib: acpi: Correct comment for HP x2 10 honor_wakeup quirk
  * mac80211: mark station unauthorized before key removal
  * nl80211: fix NL80211_ATTR_CHANNEL_WIDTH attribute type
  * scsi: sd: Fix optimal I/O size for devices that change reported values
  * scripts/dtc: Remove redundant YYLOC global declaration
  * tools: Let O= makes handle a relative path with -C option
  * perf probe: Do not depend on dwfl_module_addrsym()
  * ARM: dts: omap5: Add bus_dma_limit for L3 bus
  * ARM: dts: dra7: Add bus_dma_limit for L3 bus
  * ceph: check POOL_FLAG_FULL/NEARFULL in addition to OSDMAP_FULL/NEARFULL
  * Input: avoid BIT() macro usage in the serio.h UAPI header
  * Input: synaptics - enable RMI on HP Envy 13-ad105ng
  * Input: raydium_i2c_ts - fix error codes in raydium_i2c_boot_trigger()
  * i2c: hix5hd2: add missed clk_disable_unprepare in remove
  * ftrace/x86: Anotate text_mutex split between ftrace_arch_code_modify_post_process() and ftrace_arch_code_modify_prepare()
  * sxgbe: Fix off by one in samsung driver strncpy size arg
  * dpaa_eth: Remove unnecessary boolean expression in dpaa_get_headroom
  * mac80211: Do not send mesh HWMP PREQ if HWMP is disabled
  * scsi: ipr: Fix softlockup when rescanning devices in petitboot
  * s390/qeth: handle error when backing RX buffer
  * fsl/fman: detect FMan erratum A050385
  * arm64: dts: ls1043a: FMan erratum A050385
  * dt-bindings: net: FMan erratum A050385
  * cgroup1: don't call release_agent when it is ""
  * drivers/of/of_mdio.c:fix of_mdiobus_register()
  * cpupower: avoid multiple definition with gcc -fno-common
  * nfs: add minor version to nfs_server_key for fscache
  * cgroup-v1: cgroup_pidlist_next should update position index
  * hsr: set .netnsok flag
  * hsr: add restart routine into hsr_get_node_list()
  * hsr: use rcu_read_lock() in hsr_get_node_{list/status}()
  * vxlan: check return value of gro_cells_init()
  * tcp: repair: fix TCP_QUEUE_SEQ implementation
  * r8169: re-enable MSI on RTL8168c
  * net: phy: mdio-mux-bcm-iproc: check clk_prepare_enable() return value
  * net: dsa: mt7530: Change the LINK bit to reflect the link status
  * net: ip_gre: Accept IFLA_INFO_DATA-less configuration
  * net: ip_gre: Separate ERSPAN newlink / changelink callbacks
  * bnxt_en: Reset rings if ring reservation fails during open()
  * bnxt_en: fix memory leaks in bnxt_dcbnl_ieee_getets()
  * slcan: not call free_netdev before rtnl_unlock in slcan_open
  * NFC: fdp: Fix a signedness bug in fdp_nci_send_patch()
  * net: stmmac: dwmac-rk: fix error path in rk_gmac_probe
  * net_sched: keep alloc_hash updated after hash allocation
  * net_sched: cls_route: remove the right filter from hashtable
  * net: qmi_wwan: add support for ASKEY WWHC050
  * net/packet: tpacket_rcv: avoid a producer race condition
  * net: mvneta: Fix the case where the last poll did not process all rx
  * net: dsa: Fix duplicate frames flooded by learning
  * net: cbs: Fix software cbs to consider packet sending time
  * mlxsw: spectrum_mr: Fix list iteration in error path
  * macsec: restrict to ethernet devices
  * hsr: fix general protection fault in hsr_addr_is_self()
  * geneve: move debug check after netdev unregister
  * Revert "drm/dp_mst: Skip validating ports during destruction, just ref"
  * mmc: sdhci-tegra: Fix busy detection by enabling MMC_CAP_NEED_RSP_BUSY
  * mmc: sdhci-omap: Fix busy detection by enabling MMC_CAP_NEED_RSP_BUSY
  * mmc: core: Respect MMC_CAP_NEED_RSP_BUSY for eMMC sleep command
  * mmc: core: Respect MMC_CAP_NEED_RSP_BUSY for erase/trim/discard
  * mmc: core: Allow host controllers to require R1B for CMD6
  * f2fs: fix to avoid potential deadlock
  * f2fs: add missing function name in kernel message
  * f2fs: recycle unused compress_data.chksum feild
  * f2fs: fix to avoid NULL pointer dereference
  * f2fs: fix leaking uninitialized memory in compressed clusters
  * f2fs: fix the panic in do_checkpoint()
  * f2fs: fix to wait all node page writeback
  * mm/swapfile.c: move inode_lock out of claim_swapfile
  * fscrypt: don't evict dirty inodes after removing key
