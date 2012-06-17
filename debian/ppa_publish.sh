#!/bin/bash
# Build and publish Ubuntu/Debian source packages for several releases
# without manually editing debian/changelog each time
#
# Written by TJ <ubuntu@tjworld.net> July 2008
#
if [ $# -eq 0 ]; then
	echo -e "\nUsage: $(basename $0) dput-ppa release-a [[release-b] [release-c]]\n"
	echo -e " \tE.g. $(basename $0) my-ppa gutsy hardy intrepid\n"
	echo "Edits the changelog before calling debuild and dput to publish"
	echo -e "the source package to the buildd system for each release\n"
	echo -e "\tRun from the package source directory\n"
else
	DPUT_TARGET=$1
	PKG="$(basename $PWD)"
	echo "Publishing $PKG to dput target $DPUT_TARGET"
	shift
	cp debian/changelog /tmp/changelog.$PKG
	echo "Backed up debian/changelog to /tmp/changelog.$PKG"
	echo "Original version: $(head -n 1 debian/changelog)"
	for RELEASE in $@; do
		VER=${RELEASE:0:1}
		echo -ne "Building for $RELEASE ($VER):\t"
		sed -i -e "1,1 s/^\(.*~ppa[0-9]\+\)\(.*\)\().*\)$/\1${VER}\3/" -e "1,1 s/^\(.*) \)\(.*\)\(;.*\)$/\1${RELEASE}\3/"  debian/changelog
		echo "$(head -n 1 debian/changelog)"

		CONTROL_MODIFIED=0
		if [ -f debian/control.${RELEASE} ]; then
			mv debian/control /tmp/control.${PACKAGE}
			mv debian/control.${RELEASE} debian/control
			CONTROL_MODIFIED=1
			echo "Using debian/control.${RELEASE}"
		fi
		export BUILD_TEST=1

		debuild -i -I -S -sa > /tmp/debuild.log
		CHANGES="$(sed -n 's/^.*signfile \(.*\.changes\).*$/\1/p' /tmp/debuild.log)"
		echo -e "\nPublishing to $DPUT_TARGET with ../$CHANGES"
		dput $DPUT_TARGET ../$CHANGES
		rm ../$(basename $CHANGES _source.changes)*
		# rm /tmp/debuild.log

		if [ $CONTROL_MODIFIED -eq 1 ]; then
			# put the original control back
			mv debian/control debian/control.${RELEASE}
			mv /tmp/control.${PACKAGE} debian/control
			CONTROL_MODIFIED=0
			echo "Retire debian/control.${RELEASE}"
		fi
	done
	cp /tmp/changelog.$PKG debian/changelog
	rm -f /tmp/changelog.$PKG
	echo "debian/changelog version reset to $(head -n 1 debian/changelog)"
fi
