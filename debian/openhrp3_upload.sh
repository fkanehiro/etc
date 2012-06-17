#!/bin/sh

\cp ./debian/control* ./debian/changelog openhrp-3.1.3/debian/
cd openhrp-3.1.3; ../ppa_publish.sh ppa:hrg/release lucid maverick natty oneiric precise




