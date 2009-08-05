#include <stdio.h>
#include <stdlib.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <Defn.h>

class ImagePair: public Portable {
  public:
		ImageOf<PixelFloat> u_vels;
		ImageOf<PixelFloat> v_vels;

		virtual bool write(ConnectionWriter& connection) {
			u_vels.write(connection);
			v_vels.write(connection);
		}

		virtual bool read(ConnectionReader& connection) {
			u_vels.read(connection);
			v_vels.read(connection);
		}
};
