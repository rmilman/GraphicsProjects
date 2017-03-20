bool LensElement::intersect(const Ray &r, Vector3D *hit_p) const {
  // Part 1 Task 1: Implement this. It intersects r with this spherical lens elemnent 
  // (or aperture diaphragm). You'll want to reuse some sphere intersect code.

  //if aperature element: plane intersection else sphere element
  Vector3D cen = Vector3D(0,0,center);
  Vector3D test_hit = Vector3D(0,0,0);
  double t = 0.0;
  if (radius == 0){
    double t = (center-r.o[2])/r.d[2]; //intersects plane
    cout<<"t "<< t <<endl;
    if (t<r.min_t || t > r.max_t) return false; //within ray bounds
    test_hit = r.o+r.d*t;
    if ((test_hit-cen).norm()>aperture/2)return false;
    *hit_p = test_hit;
   // cout<<"hit"<<endl;
    return true;
  }else{
    //cout<<"not aperture"<<endl;
  }
  
  //bool test = true;
  //Vector3D cen = Vector3D(0,0,center);
  double a = dot(r.d,r.d);
  double b = dot(2*(r.o-cen), r.d);
  double c = dot((r.o-cen),(r.o-cen))-radius*radius;
  double d = b*b-4*a*c;
  if (d<0){
    cout<<"sphere miss"<<endl;
    return false;
  }
    double tp = (-b+sqrt(d))/(2*a);
    double tm = (-b-sqrt(d))/(2*a);
    double t1 = min(tp,tm);
    double t2 = max(tp,tm);
    //cout<<t1<<"  "<<t2<<endl;
    
  if (radius>0){
    t = t2;
  }else{
    t = t1;  
  }
  test_hit = r.o+r.d*t;
  //if(test){
    if (t<r.min_t || t > r.max_t){
      //cout<<"sphere miss"<<endl;
      return false;
    } 
    else if (abs(test_hit[2])>aperture/2){
      //cout<<"sphere miss"<<endl;
      return false;
    } 
    else {
      *hit_p = test_hit;
      cout<<"t "<< t <<endl;
      //cout<<"sphere hit"<<endl;
      return true;
    }
  //}else{
  //  return false;
  //}

  return false;
}