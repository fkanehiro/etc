TARGET = openrtm-aist-1.1.0
all: $(TARGET) $(TARGET)-1.deb $(TARGET)-1-src.deb

$(TARGET):
	tar jxvf OpenRTM-aist-1.1.0-RELEASE.tar.bz2
	mv OpenRTM-aist-1.1.0 openrtm-aist-1.1.0

$(TARGET)-1.deb: $(TARGET)
	\cp -r ./debian $(TARGET)
#	cd $(TARGET); fakeroot debian/rules binary

$(TARGET)-1-src.deb: $(TARGET)
	cd $(TARGET)/debian; debuild -S -sa
	\cp ./debian/control* ./debian/changelog $(TARGET)/debian/
	cd $(TARGET); ../ppa_publish.sh ppa:hrg/unstable lucid maverick natty oneiric precise

clean:
	-rm -rf $(TARGET) *~ debian/*~ *.build *.dsc *.tar.gz *.changes





