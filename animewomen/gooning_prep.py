import numpy

goon_list = {"yashers_b", "lony_tang", "bassim_cs"}

fout = open("goon_list.txt", "w")

for each in goon_list:
    print ("I love " + each + "!!!!")
    
for i in range (5000):
    for each in goon_list:
        fout.write("I F*CKING LOVE " + each + "!!!!!!!!!!\n")
        print(i)


fout.close()



