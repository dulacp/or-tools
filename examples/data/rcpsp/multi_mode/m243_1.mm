************************************************************************
file with basedata            : cm243_.bas
initial value random generator: 4867
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  106
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       35        8       35
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        2          2           8  11
   3        2          3           5   6  10
   4        2          3           5   7   8
   5        2          2          13  14
   6        2          3           7  12  13
   7        2          3           9  15  17
   8        2          3          15  16  17
   9        2          1          11
  10        2          2          11  13
  11        2          1          14
  12        2          2          14  17
  13        2          1          16
  14        2          1          16
  15        2          1          18
  16        2          1          18
  17        2          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     1       8    0    3    6
         2     2       7    0    3    4
  3      1     6       5    0    2    4
         2    10       0    7    2    3
  4      1     2       2    0    3    9
         2     3       0    4    3    7
  5      1     5       0    7    1    4
         2     5       8    0    2    6
  6      1     4       0    9    8    8
         2    10       0    9    6    2
  7      1     5       8    0    6    8
         2     6       0    8    3    3
  8      1     6       0    3    3    5
         2     6       0    5    4    2
  9      1     7       0    9    5    4
         2     7      10    0    8    8
 10      1     4       0    3    6    6
         2     7       0    2    5    6
 11      1     6       0   10    2   10
         2     6       0    8    3    9
 12      1     4       0    5    4    5
         2     5       0    5    4    3
 13      1     1       0    8    2    9
         2     4       4    0    2    6
 14      1     6       3    0    5    9
         2    10       0    6    4    6
 15      1     2       8    0    3    8
         2     9       7    0    3    7
 16      1     1       0    8    3    5
         2     6       0    6    2    2
 17      1     3       6    0    6    6
         2    10       0    5    1    4
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   20   18   59   92
************************************************************************
