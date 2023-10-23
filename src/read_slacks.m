fn=fopen('tarpaulin_currents.txt');
a=fscanf(fn,'%c');
lf=find(double(a)==10);
istart=[1 lf+1];
iend=[lf-2 length(a)];
ii=0;
istart(end)=[];

for i=1:length(istart),if strcmp(a(istart(i)+[24:27]),'0.00'),ii=ii+1;dn(ii)=datenum(a(istart(i)+[0:10 12:16]),'yyyy-mm-dd HH:MM');end,end
second_slack=round((dn-datenum('1/1/1970'))*86400);
fout=fopen('slack_times_tarpaulin.txt','w');
for i=1:length(second_slack),fprintf(fout,'%14i\n',second_slack(i));end
fclose(fout);