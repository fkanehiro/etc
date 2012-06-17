#
# Regular cron jobs for the openhrp3 package
#
0 4	* * *	root	[ -x /usr/bin/openhrp3_maintenance ] && /usr/bin/openhrp3_maintenance
