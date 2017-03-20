echo "Pixel Count Lambertian Spheres"
for i in `cat sample_sizes.txt`;
do ./pathtracer -t 8 -s ${i} -l 4 -m 5 -r 480 360 -f ../website/images/part4_bunny_${i}_4_5.png ../dae/sky/CBbunny.dae;
done
echo "Running renders of different depths, using 1024 samples per pixel and 4 samples per light."
for i in `seq 0 8`;
  do ./pathtracer -t 8 -s 1024 -l 4 -m ${i} -r 480 360 -f ../website/images/part5_spheres_1024_4_${i}_i24.png ../dae/sky/CBspheres.dae;
done
./pathtracer -t 8 -s 1024 -l 4 -m 100 -r 480 360 -f ../website/images/part5_spheres_1024_4_100_i24.png ../dae/sky/CBspheres.dae
echo "About to run renders of different samples per pixel"
for i in `cat sample_sizes.txt`;
do ./pathtracer -t 8 -s ${i} -l 1 -m 100 -r 480 360 -f ../website/images/part5_spheres_${i}_1_100_i24.png ../dae/sky/CBspheres.dae;
done

