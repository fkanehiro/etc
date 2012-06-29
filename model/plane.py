def gen_plane(w, h, dl=0.1):
    nx = int(w/dl)
    ny = int(h/dl)
    print "#point"
    for i in range(-nx,nx+1):
        for j in range(-ny, ny+1):
            print i*dl,j*dl,"0,"
    print "#coordIndex"
    for i in range(nx*2):
        for j in range(ny*2):
            idx = i*(ny*2+1)+j
            print idx,",",idx+(ny*2+1),",",idx+(ny*2+1)+1,",",idx+1,", -1,"
    print "normalIndex"
    for i in range(nx*2):
        for j in range(ny*2):
            print "0,0,0,-1,"

    
gen_plane(2,2)
    
