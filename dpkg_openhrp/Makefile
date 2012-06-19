TARGET = openhrp-3.1.3
all: $(TARGET) $(TARGET)-1.deb $(TARGET)-1-src.deb

$(TARGET):
	svn export https://openrtp.jp/svn/hrg/openhrp/3.1/trunk $(TARGET)

$(TARGET)-1.deb: $(TARGET)
	\cp -r ./debian $(TARGET)
#	cd $(TARGET); fakeroot debian/rules binary

$(TARGET)-1-src.deb: $(TARGET)
	cd $(TARGET)/debian; debuild -S -sa
	\cp ./debian/control* ./debian/changelog $(TARGET)/debian/
	cd $(TARGET); ../ppa_publish.sh ppa:hrg/unstable lucid maverick natty oneiric precise

clean:
	-rm -rf $(TARGET) *~ debian/*~ *.build *.dsc *.tar.gz *.changes





