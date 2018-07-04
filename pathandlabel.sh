# /usr/bin/env sh
#run from home
DATA=/home/liquan/QtProjects/vibe_kcf/vibe_kcf_svm_opencv3
echo "Create tes.txt..."
#rm -rf $DATA/*.txt
find $DATA/build-vibe_kcf_svm_opencv3-Desktop_Qt_5_8_0_GCC_64bit-Debug -name *.jpg  >>$DATA/test.txt
#find $DATA/animals -name *.jpg  | cut -d '/' -f1|sed "s/$/ 0/" >>$DATA/train.txt
#find $DATA/car -name *.jpg | cut -d '/' -f1-9|sed "s/$/\n1/" >>$DATA/train.txt
#find $DATA/car -name *.jpg  | cut -d '/' -f1|sed "s/$/ 1/" >>$DATA/train.txt
#find $DATA/person -name *.jpg | cut -d '/' -f1-9|sed "s/$/\n2/" >>$DATA/train.txt
#find $DATA/testimg -name *.jpg | cut -d '/' -f1-9 >>$DATA/myImagetest.txt
#find $DATA/Negative -name *.jpg | cut -d '/' -f1-9|sed "s/$/\n3/" >>$DATA/train.txt

#cat $DATA/*.txt>>$DATA/train_all.txt
#sudo awk 'BEGIN{ 100000*srand();}{ printf "%s %s\n", rand(), $0}'  $DATA/train_all.txt |sort -k1n | awk '{gsub($1FS,""); print $0}' | tee $DATA/traindata.txt 
chmod 777 $DATA/*.txt
#get the file length and save as var 'a'
#a=$(echo $((wc -l $DATA/traindata.txt) | cut -d ' ' -f 1))
#echo NumTrain=$(echo $((wc -l $DATA/train.txt) | cut -d ' ' -f 1))
#echo NumVal=$(echo $((wc -l $DATA/val.txt) | cut -d ' ' -f 1))

#trainNum=$(((a*7)/10))
#echo trainNums=$(((a*7)/10))
#split -l $trainNum　$DATA/traindata.txt $DATA/traindata1.txt
echo "Done.."
