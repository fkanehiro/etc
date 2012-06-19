TARGET = libeigen3-dev-3.0.5
all: $(TARGET) $(TARGET)-1.deb $(TARGET)-1-src.deb

$(TARGET):
	tar jxvf eigen-eigen-6e7488e20373.tar.bz2
	mv eigen-eigen-6e7488e20373 libeigen3-dev-3.0.5

$(TARGET)-1.deb: $(TARGET)
	\cp -r ./debian $(TARGET)
	#cd $(TARGET); fakeroot debian/rules binary

$(TARGET)-1-src.deb: $(TARGET)
	cd $(TARGET)/debian; debuild -S -sa
	\cp ./debian/control* ./debian/changelog $(TARGET)/debian/
	cd $(TARGET); ../ppa_publish.sh ppa:hrg/unstable maverick natty
	#cd $(TARGET); ../ppa_publish.sh ppa:hrg/unstable lucid

clean:
	-rm -rf $(TARGET) *~ debian/*~ *.build *.dsc *.tar.gz *.changes *.deb





