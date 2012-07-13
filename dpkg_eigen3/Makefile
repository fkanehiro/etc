TARGET = libeigen3-dev-3.0.6
PPA = ppa:hrg/unstable
all: $(TARGET) $(TARGET)-1.deb $(TARGET)-1-src.deb

$(TARGET):
	tar jxvf eigen-eigen-c76e08cca8eb.tar.bz2
	mv eigen-eigen-c76e08cca8eb libeigen3-dev-3.0.6

$(TARGET)-1.deb: $(TARGET)
	\cp -r ./debian $(TARGET)
	#cd $(TARGET); fakeroot debian/rules binary

$(TARGET)-1-src.deb: $(TARGET)
	dch -i -c ./debian/changelog
	cd $(TARGET)/debian; debuild -S -sa
	\cp ./debian/control* ./debian/changelog $(TARGET)/debian/
	cd $(TARGET); ../ppa_publish.sh $(PPA) lucid maverick natty

clean:
	-rm -rf $(TARGET) *~ debian/*~ *.build *.dsc *.tar.gz *.changes *.deb





