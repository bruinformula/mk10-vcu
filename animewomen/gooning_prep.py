import numpy

goon_list = {"yashers_b", "lony_tang", "bassim_cs", "kadin_m", "randih"}

fout = open("goon_list2.txt", "w")

for each in goon_list:
    print ("I love " + each + "!!!!")
    
for i in range (20000):
    for each in goon_list:
        fout.write("I F*CKING LOVE " + each + "!!!!!!!!!!\n")
        print(i)


fout.close()



