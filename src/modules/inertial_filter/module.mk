
MODULE_COMMAND	 	= inertial_filter
SRCS		 	= inertial_filter_main.c \
			inertial_filter.c

MODULE_STACKSIZE = 1200

EXTRACFLAGS = -Wframe-larger-than=3500

