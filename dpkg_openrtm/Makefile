TARGET = openrtm-aist-1.1.0
PPA = ppa:hrg/unstable
all: $(TARGET) $(TARGET)-1.deb $(TARGET)-1-src.deb

$(TARGET):
	tar jxvf OpenRTM-aist-1.1.0-RELEASE.tar.bz2
	mv OpenRTM-aist-1.1.0 $(TARGET)
	cp SDOPackage.idl $(TARGET)/src/lib/rtm/idl

$(TARGET)-1.deb: $(TARGET)
	\cp -r ./debian $(TARGET)
#	cd $(TARGET); fakeroot debian/rules binary

$(TARGET)-1-src.deb: $(TARGET)
	dch -i -c ./debian/changelog
	cd $(TARGET)/debian; debuild -S -sa
	\cp ./debian/control* ./debian/changelog $(TARGET)/debian/
	cd $(TARGET); ../ppa_publish.sh $(PPA) lucid maverick natty oneiric precise quantal

clean:
	-rm -rf $(TARGET) *~ debian/*~ *.build *.dsc *.tar.gz *.changes





