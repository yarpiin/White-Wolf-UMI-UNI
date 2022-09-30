
n.n.n / 2022-09-30
==================

  * simple_lmk: Set the victims to the highest priority
  * schedutil: optimize with compiler to check WALT use.
  * cpufreq: schedutil: fix check for stale utilization values
  *  soc: qcom: smp2p_sleepstate: Add suspend delay
  * touchscreen: fts_521: Fix strict-prototypes errors
  * touchscreen: goodix_driver_gt9886: Fix strict-prototypes errors
  * msm: adsprpc: Fix race condition in internal_control
  * ARM64: dts: j2:  Bring back Xiaomi FOD dimlayer impl
  * ARM64: dts: J2: Skip entering NOLP while FOD on
  * defconfigs: regen
  * version 16
  * defconfig: regen
  * Merge tag 'LA.UM.9.12.r1-14700-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14700-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/display-drivers into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14700-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14700.01-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * techpack: display:  Implement doze mode
  * techpack: display: remove stock doze implementation
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
  * disp: msm: Revert all panel power state modification changes
  * kernel: time: reduce ntp wakeups
  * scripts/Makefile.lib: Speed up build process
  * cpufreq: Ensure the minimal frequency is lower than the maximal frequency
  * techpack: display: dsi_phy: Force allow panel phy power off
  * techpack: display: dsi_panel: Force enable Ultra-Low Power State feature for all panel
  * power_supply: don't allocate attrname
  * regulator: Retry read and write i2c operations on failure
  * arm64: dts: kona: Switch to step-wise thermal governor
  * BACKPORT: msm: kgsl: Add support for KGSL_PROP_VK_DEVICE_ID
  * BACKPORT: msm: kgsl: Add a property to query gpu model name
  * BACKPORT: msm: kgsl: Add support to get gpu model from device tree
  * rmnet_ipa: Fix netdev watchdog triggering on suspend
  * msm: ipa: Do not acquire wakelocks
  * drivers: android: Place a no-op PSI driver for SLMK
  * arm64: dts: umi: optimise battery charging further
  * mbcache: Speed up cache entry creation
  * arm64: select HAVE_MOVE_PMD for faster mremap
  * mm: speed up mremap by 20x on large regions
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
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
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * cpufreqX: schedutil: Use kobject release() method to free sugov_tunables
  * arch: dts: Throttle charging speed and Optimise charging current
  * arm64: lib: Memory utilities optimization
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * setlocalversion: Never append a "+" to localversion
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
  * ARM64: kona/lito: Optimize FOD HBM to fix fod flashing
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
  * techpack: display: msm: dsi: set doze brightness for aod correctly
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
  * qcacld-3.0: Guard K81/K81A specific wlan modifications
  * qcacld-3.0: Import xiaomi modifications from psyche-r-oss
  * qcacld-3.0: Fix race during peer deletion in roaming scenario
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
  * Revert ".gitignore: Add device tree vendor directories to gitignore"
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
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/video-driver into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/display-drivers into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/camera-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * techpack: Remove .gitignore
  * Revert ".gitignore: Add techpack directory to gitignore"
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * initial commit

n.n.n / 2022-09-23
==================

  * defconfig: regen
  * Merge tag 'LA.UM.9.12.r1-14700-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14700-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/display-drivers into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14700-SMxx50.QSSI13.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14700-SMxx50.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge 367074bf05bc4eecfdb4ee3f356f34312efc0d00 on remote branch
  *  version 16
  * techpack: display:  Implement doze mode
  * techpack: display: remove stock doze implementation
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
  * disp: msm: Revert all panel power state modification changes
  * kernel: time: reduce ntp wakeups
  * scripts/Makefile.lib: Speed up build process
  * cpufreq: Ensure the minimal frequency is lower than the maximal frequency
  * techpack: display: dsi_phy: Force allow panel phy power off
  * techpack: display: dsi_panel: Force enable Ultra-Low Power State feature for all panel
  * power_supply: don't allocate attrname
  * regulator: Retry read and write i2c operations on failure
  * arm64: dts: kona: Switch to step-wise thermal governor
  * BACKPORT: msm: kgsl: Add support for KGSL_PROP_VK_DEVICE_ID
  * BACKPORT: msm: kgsl: Add a property to query gpu model name
  * BACKPORT: msm: kgsl: Add support to get gpu model from device tree
  * rmnet_ipa: Fix netdev watchdog triggering on suspend
  * msm: ipa: Do not acquire wakelocks
  * drivers: android: Place a no-op PSI driver for SLMK
  * arm64: dts: umi: optimise battery charging further
  * mbcache: Speed up cache entry creation
  * arm64: select HAVE_MOVE_PMD for faster mremap
  * mm: speed up mremap by 20x on large regions
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
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
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * cpufreqX: schedutil: Use kobject release() method to free sugov_tunables
  * arch: dts: Throttle charging speed and Optimise charging current
  * arm64: lib: Memory utilities optimization
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * setlocalversion: Never append a "+" to localversion
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
  * ARM64: kona/lito: Optimize FOD HBM to fix fod flashing
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
  * techpack: display: msm: dsi: set doze brightness for aod correctly
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
  * qcacld-3.0: Guard K81/K81A specific wlan modifications
  * qcacld-3.0: Import xiaomi modifications from psyche-r-oss
  * qcacld-3.0: Fix race during peer deletion in roaming scenario
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
  * Revert ".gitignore: Add device tree vendor directories to gitignore"
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
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/video-driver into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/display-drivers into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/camera-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * techpack: Remove .gitignore
  * Revert ".gitignore: Add techpack directory to gitignore"
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * initial commit
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
  * msm: kona_defconfig: Added the flag for ICE Driver as required by FDE
  * mmc: Enable Inline Crypto Engine clocks before programming key
  * f2fs: guarantee to write dirty data when enabling checkpoint back
  * f2fs: flush data when enabling checkpoint back

n.n.n / 2022-09-03
==================

  *  version 16
  * techpack: display:  Implement doze mode
  * techpack: display: remove stock doze implementation
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
  * disp: msm: Revert all panel power state modification changes
  * kernel: time: reduce ntp wakeups
  * scripts/Makefile.lib: Speed up build process
  * cpufreq: Ensure the minimal frequency is lower than the maximal frequency
  * techpack: display: dsi_phy: Force allow panel phy power off
  * techpack: display: dsi_panel: Force enable Ultra-Low Power State feature for all panel
  * power_supply: don't allocate attrname
  * regulator: Retry read and write i2c operations on failure
  * arm64: dts: kona: Switch to step-wise thermal governor
  * BACKPORT: msm: kgsl: Add support for KGSL_PROP_VK_DEVICE_ID
  * BACKPORT: msm: kgsl: Add a property to query gpu model name
  * BACKPORT: msm: kgsl: Add support to get gpu model from device tree
  * rmnet_ipa: Fix netdev watchdog triggering on suspend
  * msm: ipa: Do not acquire wakelocks
  * drivers: android: Place a no-op PSI driver for SLMK
  * arm64: dts: umi: optimise battery charging further
  * mbcache: Speed up cache entry creation
  * arm64: select HAVE_MOVE_PMD for faster mremap
  * mm: speed up mremap by 20x on large regions
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
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
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * cpufreqX: schedutil: Use kobject release() method to free sugov_tunables
  * arch: dts: Throttle charging speed and Optimise charging current
  * arm64: lib: Memory utilities optimization
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * setlocalversion: Never append a "+" to localversion
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
  * ARM64: kona/lito: Optimize FOD HBM to fix fod flashing
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
  * techpack: display: msm: dsi: set doze brightness for aod correctly
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
  * qcacld-3.0: Guard K81/K81A specific wlan modifications
  * qcacld-3.0: Import xiaomi modifications from psyche-r-oss
  * qcacld-3.0: Fix race during peer deletion in roaming scenario
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
  * Revert ".gitignore: Add device tree vendor directories to gitignore"
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
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/video-driver into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/display-drivers into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/camera-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * techpack: Remove .gitignore
  * Revert ".gitignore: Add techpack directory to gitignore"
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * initial commit
  * Merge 227ed6d234e0d6a89c26d60abd1e424fd7b83111 on remote branch
  * Merge "drivers: thermal: Add valid cpu check in cpu isolate driver"
  * drivers: thermal: Add valid cpu check in cpu isolate driver
  * msm: kgsl: Use consolidated power level for thermal limit
  * msm: kona_defconfig: Added the flag for ICE Driver as required by FDE

n.n.n / 2022-09-01
==================

  * xx
  * techpack: display:  Implement doze mode
  * techpack: display: remove stock doze implementation
  * techpack: display: temporary disable DC dimming when in fod_hbm mode
  * techpack: display: adapt exposure adjustment methods to sm8250
  * techpack: display: Let exposure adjustment switchable via sysfs
  * techpack: display: Introduce exposure adjustment driver
  * dsi_display:  Use type_map array index 0 for invalid hbm values
  * dsi_display: Add support for high brightness mode (HBM)
  * techpack: display: msm: dsi: create empty attributes group
  * cnss2: Avoid race condition between time sync and system suspend
  * cnss2: Make sure the write to remap window register take effect
  * touchscreen: nt36672c: Return 0 if allocation fails in test_buff_init
  * touchscreen: nt36672c: Fix unable to set xiaomi touch interface mode
  * touchscreen: goodix_driver_gt9886: Fix memory leak in goodix_cfg_bin_proc
  * input: misc: aw8697: Fix memory leak in aw8697_file_write
  * disp: msm: Revert all panel power state modification changes
  * kernel: time: reduce ntp wakeups
  * scripts/Makefile.lib: Speed up build process
  * cpufreq: Ensure the minimal frequency is lower than the maximal frequency
  * techpack: display: dsi_phy: Force allow panel phy power off
  * techpack: display: dsi_panel: Force enable Ultra-Low Power State feature for all panel
  * power_supply: don't allocate attrname
  * regulator: Retry read and write i2c operations on failure
  * arm64: dts: kona: Switch to step-wise thermal governor
  * BACKPORT: msm: kgsl: Add support for KGSL_PROP_VK_DEVICE_ID
  * BACKPORT: msm: kgsl: Add a property to query gpu model name
  * BACKPORT: msm: kgsl: Add support to get gpu model from device tree
  * rmnet_ipa: Fix netdev watchdog triggering on suspend
  * msm: ipa: Do not acquire wakelocks
  * drivers: android: Place a no-op PSI driver for SLMK
  * arm64: dts: umi: optimise battery charging further
  * mbcache: Speed up cache entry creation
  * arm64: select HAVE_MOVE_PMD for faster mremap
  * mm: speed up mremap by 20x on large regions
  * drivers: xiaomi_touch: add a sysfs node to bump touch sample rate
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
  * usb: dwc3-msm: Set usb_data_enabled initial value earlier
  * usb: new attributes implementation to enable/disable usb data Bug: 188760285 Test: driver probe and attributes access normally Signed-off-by: Albert Wang <albertccwang@google.com> Change-Id: I0aec98eebff9454cdec065bb09825f6442ac013b
  * cpufreqX: schedutil: Use kobject release() method to free sugov_tunables
  * arch: dts: Throttle charging speed and Optimise charging current
  * arm64: lib: Memory utilities optimization
  * thermal: tsens: remove unused
  * drivers: arch_topology: wire up thermal limit for arch_scale_max_freq_capacity
  * net: ipv4: Reduce TCP performance spikes
  * setlocalversion: Never append a "+" to localversion
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
  * ARM64: kona/lito: Optimize FOD HBM to fix fod flashing
  * techpack: display: sde_connector: update FOD HBM on dim layer change
  * techpack: display: sde_crtc: setup dim layer
  * techpack: display: sde_plane: expose method to check if plane is FOD
  * techpack: display: dsi_panel: expose attribute to notify of fod changes
  * techpack: display: dsi_panel: expose FOD dim and method to set FOD HBM
  * techpack: display: dsi_panel: track real backlight value
  * techpack: display: dsi_panel: read FOD dimming LUT
  * techpack: display: sde_hw_mdss: Increase sde stage to map zpos changes
  * teckpack: display: dsi_panel: create empty ettributes group
  * techpack: display: dsi_display: expose method to get main display
  * techpack: display: sde_plane: extract bit in ZPOS into FOD
  * techpack: display: sde_plane: add FOD prop
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
  * techpack: display: msm: dsi: set doze brightness for aod correctly
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
  * bpf: Get the comm hash of the socket process stored inside sk buffer
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
  * qcacld-3.0: Guard K81/K81A specific wlan modifications
  * qcacld-3.0: Import xiaomi modifications from psyche-r-oss
  * qcacld-3.0: Fix race during peer deletion in roaming scenario
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
  * Revert ".gitignore: Add device tree vendor directories to gitignore"
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
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/video-driver into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/display-drivers into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/camera-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/data-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/opensource/audio-kernel into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/fw-api into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/platform/vendor/qcom-opensource/wlan/qcacld-3.0 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * techpack: Remove .gitignore
  * Revert ".gitignore: Add techpack directory to gitignore"
  * Merge tag 'LA.UM.9.12.r1-14600-SMxx50.QSSI12.0' of https://git.codelinaro.org/clo/la/kernel/msm-4.19 into LA.UM.9.12.r1-XXXX-SMxx50.QSSI12.0-new
  * initial commit
  * Merge 227ed6d234e0d6a89c26d60abd1e424fd7b83111 on remote branch
  * Merge "drivers: thermal: Add valid cpu check in cpu isolate driver"
  * drivers: thermal: Add valid cpu check in cpu isolate driver
  * msm: kgsl: Use consolidated power level for thermal limit
  * msm: kona_defconfig: Added the flag for ICE Driver as required by FDE
