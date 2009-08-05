
//#include <list>
//using namespace std;

void label2image(ImageOf<PixelInt>& src, ImageOf<PixelRgb>& dest) {
  dest.resize(src);
  //hash_ii idx;
  //list<PixelRgb> pens;

  IMGFOR(src,x,y) {
    int id = src(x,y);
    //if (idx.find(id)==idx.end()) {

    dest(x,y).r = (id*341)%200+50;
    dest(x,y).g = (id*9707)%256;
    dest(x,y).b = (id*914)%256;
    //idx = pens.size();
    //PixelRgb color(id
    //pens.push_back(color);
    
  }
}
