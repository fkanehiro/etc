TARGET = openrtm-aist-python-1.1.0
PPA = ppa:hrg/unstable
all: $(TARGET) $(TARGET)-1.deb $(TARGET)-1-src.deb

$(TARGET):
	tar zxvf OpenRTM-aist-Python-1.1.0-RC1.tar.gz
	mv OpenRTM-aist-Python-1.1.0 $(TARGET)
#	cp Makefile.openrtm $(TARGET)/Makefile

$(TARGET)-1.deb: $(TARGET)
	\cp -r ./debian $(TARGET)
#	cd $(TARGET); fakeroot debian/rules binary

$(TARGET)-1-src.deb: $(TARGET)
	cd $(TARGET)/debian; debuild -S -sa
	\cp ./debian/control* ./debian/changelog $(TARGET)/debian/
	cd $(TARGET); ../ppa_publish.sh $(PPA) lucid maverick natty oneiric precise

clean:
	-rm -rf $(TARGET) *~ debian/*~ *.build *.dsc *.tar.gz *.changes





