#! /bin/bash
#打印输出
printf " my name is %-5s ,my borth is  :%10s: \n" sanchuan Mark
echo -e "1\t2\t3\t"
# echo -e "\e[1;31red txt \e[0m"
echo -e "\e[1;41mthis is \e[0m  heiheihei"
echo "hualalal"

# 变量和环境变量
 a=1;b=2;
echo $a+$b
echo ${a}+b 汉语支持吗
echo "nishi ${a} woshi ${b},$a,$a,$b,$b"

var="1234567"
echo ${#var}

#数学运算
no1=4;no2=3
let result=no1+no2
echo $result
let no1++
let no2--
echo $no1 $no2
let no1+=6
let no2-=10
echo $no1 $no2
echo $[no1+no2]
echo $[no1+5]
echo "$no1*$no2"
echo "$no1*$no2"|bc 
result=`echo "$no1*$no2"|bc`
echo $result

# 数组
arr=(1 2 3 4 5 6)
echo ${arr[5]}
echo ${arr[*]}

arr[6]="adsasdasd"
echo ${arr[@]}
declare -A ass_array
ass_array[apple]='一百块'
ass_array[orange]='十块钱'
echo ${ass_array[*]}