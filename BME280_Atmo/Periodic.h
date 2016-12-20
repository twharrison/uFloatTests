struct periodic_info {
	int timer_fd;
	unsigned long long wakeups_missed;
};

static int make_periodic(unsigned int period, struct periodic_info *info);

static void wait_period(struct periodic_info *info);