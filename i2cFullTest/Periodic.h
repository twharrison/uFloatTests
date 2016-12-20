struct periodic_info {
	int timer_fd;
	unsigned long long wakeups_missed;
};

int make_periodic(unsigned int period, struct periodic_info *info);

void wait_period(struct periodic_info *info);