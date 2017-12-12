This tree includes a GPLv2 cross-compiler, linux kernel w/ config and
various open sources.

For more information on the Marvell armv7-marvell-linux-gnueabi-hard_i686 compiler, go to:

	http://gcc.gnu.org/gcc-4.6/

This has been tested on ubuntu 12.04 LTS desktop edition (32 bit).

In addition to the standard packages included in ubuntu, you many need:

	flex
	bison
	patch
	autoconf
	libncurses5-dev

which can be installed with 'sudo apt-get install flex bison patch autoconf libncurses5-dev'.

Note: In order to compile, you need to have an active internet connection.

To build the sources, simply run 'make'.

Some sources (openssl, zlib, ncurses) are required to compile some
components, but are not included in this distribution.  You can obtain
these from the internet.  The Makefile.src will attempt to do this, but if
it fails, because you are not connected to the internet or the packages
are not available at the locations they used to be, you may have to find
a recent version of those packages manually.
