TARGET = openhrp-3.1.3
all: $(TARGET) $(TARGET)-1.deb $(TARGET)-1-src.deb

$(TARGET):
	svn co https://openrtp.jp/svn/hrg/openhrp/3.1/trunk openhrp-3.1.3

$(TARGET)-1.deb: $(TARGET)
	\cp -r ./debian $(TARGET)
	cd $(TARGET); fakeroot debian/rules binary

$(TARGET)-1-src.deb: $(TARGET)
	cd $(TARGET)/debian; debuild -S -sa
	./openhrp3_upload.sh

clean:
	-rm -rf openhrp-3.1.3 openhrp_3.1.3* *~ debian/*~



