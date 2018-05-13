function [in] = insidepoly_halfplane(c,d)

% walls
x1 = ([-7.4 -7.4 7.4 7.4]+8)*10;
y1 = ([-4.925 +4.925 4.9925 -4.9925]+5)*10;

[in11,on11] = inpolygon(c,d,x1,y1);
[in12,on12] = inpolygon(c-2,d,x1,y1);
[in13,on13] = inpolygon(c+2,d,x1,y1);
[in14,on14] = inpolygon(c,d+2,x1,y1);
[in15,on15] = inpolygon(c,d-2,x1,y1);

in1 = (in11|in12|in13|in14|in15|on11|on12|on13|on14|on15);

% Tables 

x2 = ([-7.33 -6.55 -6.55 -7.33]+8)*10;
y2 = ([2.725 2.725  0.725 0.725]+5)*10;

[in21,on21] = inpolygon(c,d,x2,y2);
[in22,on22] = inpolygon(c-2,d,x2,y2);
[in23,on23] = inpolygon(c+2,d,x2,y2);
[in24,on24] = inpolygon(c,d+2,x2,y2);
[in25,on25] = inpolygon(c,d-2,x2,y2);

in2 = (in21|in22|in23|in24|in25|on21|on22|on23|on24|on25);


x3 = ([-7.35 -6.55 -6.55 -7.33]+8)*10;
y3 = ([0.725 0.725 -1.275 -1.275]+5)*10;

[in31,on31] = inpolygon(c,d,x3,y3);
[in32,on32] = inpolygon(c-2,d,x3,y3);
[in33,on33] = inpolygon(c+2,d,x3,y3);
[in34,on34] = inpolygon(c,d+2,x3,y3);
[in35,on35] = inpolygon(c,d-2,x3,y3);

in3 = (in31|in32|in33|in34|in35|on31|on32|on33|on34|on35);

x4 = ([-6.15 -4.55 -4.55 -6.15]+8)*10;
y4 = ([ -3.55 -3.55 -4.85 -4.85]+5)*10;

[in41,on41] = inpolygon(c,d,x4,y4);
[in42,on42] = inpolygon(c-2,d,x4,y4);
[in43,on43] = inpolygon(c+2,d,x4,y4);
[in44,on44] = inpolygon(c,d+2,x4,y4);
[in45,on45] = inpolygon(c,d-2,x4,y4);

in4 = (in41|in42|in43|in44|in45|on41|on42|on43|on44|on45);

x5 = ([ 5.125 5.925 5.925 5.125]+8)*10;
y5 = ([ -2.375 -2.375 -4.375 -4.375]+5)*10;

[in51,on51] = inpolygon(c,d,x5,y5);
[in52,on52] = inpolygon(c-2,d,x5,y5);
[in53,on53] = inpolygon(c+2,d,x5,y5);
[in54,on54] = inpolygon(c,d+2,x5,y5);
[in55,on55] = inpolygon(c,d-2,x5,y5);

in5 = (in51|in52|in53|in54|in55|on51|on52|on53|on54|on55);

x6 = ([3.925 4.725 4.725 3.925]+8)*10;
y6 = ([-2.375 -2.375 -4.375 -4.375]+5)*10;

[in61,on61] = inpolygon(c,d,x6,y6);
[in62,on62] = inpolygon(c-2,d,x6,y6);
[in63,on63] = inpolygon(c+2,d,x6,y6);
[in64,on64] = inpolygon(c,d+2,x6,y6);
[in65,on65] = inpolygon(c,d-2,x6,y6);

in6 = (in61|in62|in63|in64|in65|on61|on62|on63|on64|on65);

x7 = ([-3.4 -1.8 -1.8 -3.4]+8)*10;
y7 = ([ 1.6  1.6  -1.6 -1.6]+5)*10;

[in71,on71] = inpolygon(c,d,x7,y7);
[in72,on72] = inpolygon(c-2,d,x7,y7);
[in73,on73] = inpolygon(c+2,d,x7,y7);
[in74,on74] = inpolygon(c,d+2,x7,y7);
[in75,on75] = inpolygon(c,d-2,x7,y7);

in7 = (in71|in72|in73|in74|in75|on71|on72|on73|on74|on75);

x8 = 10*([0.3475 1.9475 1.9475 0.3475]+8);
y8= ([1.6  1.6  -1.6 -1.6]+5)*10;

[in81,on81] = inpolygon(c,d,x8,y8);
[in82,on82] = inpolygon(c-2,d,x8,y8);
[in83,on83] = inpolygon(c+2,d,x8,y8);
[in84,on84] = inpolygon(c,d+2,x8,y8);
[in85,on85] = inpolygon(c,d-2,x8,y8);

in8 = (in81|in82|in83|in84|in85|on81|on82|on83|on84|on85);

if in1
    if ~(in2 | in3 | in4 | in5 | in6 | in7 | in8)
        in = false;
    else
        in = true;
    end
else
    in = true;
end
    

end 
