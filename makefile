# 2016/5

cc	= gcc
link	= gcc

hdrs	      = color.h 

useobj  = mshade.o \
		  subfunc.o \
		  uglyfont.o

animation.exe : anima.o $(useobj)
	$(link) anima.o $(useobj) -o animation.exe -lfreeglut -lglu32 -lopengl32

subfunc.o : subfunc.c $(hdrs)
		$(cc) -c subfunc.c $(cdebug) $(cvars) -o subfunc.o

anima.o : anima.c color.h $(hdrs)
	$(cc) -c anima.c -o anima.o -lfreeglut -lglu32 -lopengl32

mshade.o : mshade.c anima.c color.h $(hdrs)
		$(cc) -c mshade.c $(cdebug) $(cvars) -o mshade.o

uglyfont.o : uglyfont.c $(hdrs)
		$(cc) -c uglyfont.c $(cdebug) $(cvars) -o uglyfont.o

clean: 
	del r_model.o mshade.o subfunc.o anima.o uglyfont.o animation.exe
