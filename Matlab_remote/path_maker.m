
size_factor= 2;

t = linspace(0,130,131);
plot(size_factor*sin(0.1*t), 2*size_factor*sin(0.05*t))
hold on
scatter(0,0,'r')
t2 = linspace(0,130,28);


 for i=1:length(t2)
    scatter(size_factor*sin(0.1*i*4.8148), 2*size_factor*sin(0.05*i*4.8148),'r')
    data_array(3*(i-1)+1)= size_factor*sin(0.1*i*4.8148);
    data_array(3*(i-1)+2)= 2*size_factor*sin(0.05*i*4.8148);
    data_array(3*(i-1)+3)= 0;
 end