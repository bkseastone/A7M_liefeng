#include "rectify.h"
#include "math.h"
        int a = 0;
        int16  RectifyX[60][81]=
        {
{-167,-164,-160,-157,-153,-149,-146,-142,-139,-135,-131,-128,-124,-121,-117,-113,-110,-106,-103,-99,-95,-92,-88,-85,-81,-78,-74,-70,-67,-63,-60,-56,-53,-49,-45,-42,-38,-35,-31,-28,-24,-21,-17,-14,-10,-7,-3,0,4,8,11,15,18,22,25,29,32,36,39,43,46,50,53,57,60,64,67,70,74,77,81,84,88,91,95,98,102,105,109,112,300},
{-150,-147,-144,-141,-137,-134,-131,-128,-124,-121,-118,-115,-111,-108,-105,-102,-98,-95,-92,-89,-86,-82,-79,-76,-73,-70,-66,-63,-60,-57,-53,-50,-47,-44,-41,-37,-34,-31,-28,-25,-22,-18,-15,-12,-9,-6,-2,1,4,7,10,13,17,20,23,26,29,32,36,39,42,45,48,51,54,58,61,64,67,70,73,76,80,83,86,89,92,95,98,101,300},
{-136,-133,-131,-128,-125,-122,-119,-116,-113,-110,-107,-104,-101,-98,-95,-92,-89,-86,-83,-80,-78,-75,-72,-69,-66,-63,-60,-57,-54,-51,-48,-45,-43,-40,-37,-34,-31,-28,-25,-22,-19,-16,-14,-11,-8,-5,-2,1,4,7,10,12,15,18,21,24,27,30,33,35,38,41,44,47,50,53,56,58,61,64,67,70,73,76,78,81,84,87,90,93,300},
{-125,-122,-119,-117,-114,-111,-109,-106,-103,-100,-98,-95,-92,-90,-87,-84,-82,-79,-76,-74,-71,-68,-66,-63,-60,-57,-55,-52,-49,-47,-44,-41,-39,-36,-33,-31,-28,-25,-23,-20,-17,-15,-12,-9,-7,-4,-2,1,4,6,9,12,14,17,20,22,25,28,30,33,35,38,41,43,46,49,51,54,56,59,62,64,67,70,72,75,77,80,83,85,300},
{-115,-112,-110,-107,-105,-102,-100,-97,-95,-93,-90,-88,-85,-83,-80,-78,-75,-73,-70,-68,-65,-63,-60,-58,-55,-53,-50,-48,-45,-43,-40,-38,-36,-33,-31,-28,-26,-23,-21,-18,-16,-13,-11,-9,-6,-4,-1,1,4,6,9,11,13,16,18,21,23,26,28,31,33,35,38,40,43,45,48,50,52,55,57,60,62,65,67,69,72,74,77,79,300},
{-107,-104,-102,-100,-97,-95,-93,-90,-88,-86,-83,-81,-79,-76,-74,-72,-70,-67,-65,-63,-60,-58,-56,-53,-51,-49,-47,-44,-42,-40,-37,-35,-33,-31,-28,-26,-24,-21,-19,-17,-15,-12,-10,-8,-5,-3,-1,1,4,6,8,10,13,15,17,20,22,24,26,29,31,33,35,38,40,42,44,47,49,51,53,56,58,60,62,65,67,69,71,74,300},
{-99,-97,-95,-93,-91,-88,-86,-84,-82,-80,-78,-75,-73,-71,-69,-67,-65,-63,-60,-58,-56,-54,-52,-50,-48,-45,-43,-41,-39,-37,-35,-33,-30,-28,-26,-24,-22,-20,-18,-16,-13,-11,-9,-7,-5,-3,-1,2,4,6,8,10,12,14,16,18,21,23,25,27,29,31,33,35,37,40,42,44,46,48,50,52,54,56,59,61,63,65,67,69,300},
{-93,-91,-89,-87,-85,-83,-81,-79,-77,-75,-73,-71,-69,-67,-65,-63,-60,-58,-56,-54,-52,-50,-48,-46,-44,-42,-40,-38,-36,-34,-32,-30,-28,-26,-24,-22,-20,-18,-16,-14,-12,-10,-8,-6,-4,-2,0,2,4,6,8,10,12,14,16,18,19,21,23,25,27,29,31,33,35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,65,300},
{-87,-85,-83,-81,-80,-78,-76,-74,-72,-70,-68,-66,-64,-62,-61,-59,-57,-55,-53,-51,-49,-47,-45,-44,-42,-40,-38,-36,-34,-32,-30,-28,-27,-25,-23,-21,-19,-17,-15,-13,-11,-10,-8,-6,-4,-2,0,2,4,5,7,9,11,13,15,17,19,20,22,24,26,28,30,32,33,35,37,39,41,43,45,47,48,50,52,54,56,58,60,61,300},
{-82,-80,-79,-77,-75,-73,-71,-70,-68,-66,-64,-62,-61,-59,-57,-55,-53,-52,-50,-48,-46,-45,-43,-41,-39,-37,-36,-34,-32,-30,-28,-27,-25,-23,-21,-20,-18,-16,-14,-12,-11,-9,-7,-5,-4,-2,0,2,4,5,7,9,11,12,14,16,18,19,21,23,25,27,28,30,32,34,35,37,39,41,42,44,46,48,49,51,53,55,56,58,300},
{-78,-76,-74,-73,-71,-69,-67,-66,-64,-62,-61,-59,-57,-56,-54,-52,-50,-49,-47,-45,-44,-42,-40,-39,-37,-35,-34,-32,-30,-28,-27,-25,-23,-22,-20,-18,-17,-15,-13,-12,-10,-8,-7,-5,-3,-2,0,2,4,5,7,9,10,12,14,15,17,19,20,22,24,25,27,29,30,32,34,35,37,39,40,42,44,45,47,49,50,52,54,55,300},
{-74,-72,-70,-69,-67,-66,-64,-62,-61,-59,-57,-56,-54,-53,-51,-49,-48,-46,-45,-43,-41,-40,-38,-37,-35,-33,-32,-30,-29,-27,-25,-24,-22,-21,-19,-17,-16,-14,-12,-11,-9,-8,-6,-4,-3,-1,0,2,3,5,7,8,10,11,13,15,16,18,19,21,23,24,26,27,29,31,32,34,35,37,38,40,42,43,45,46,48,50,51,53,300},
{-70,-68,-67,-65,-64,-62,-61,-59,-58,-56,-55,-53,-52,-50,-48,-47,-45,-44,-42,-41,-39,-38,-36,-35,-33,-32,-30,-29,-27,-25,-24,-22,-21,-19,-18,-16,-15,-13,-12,-10,-9,-7,-6,-4,-3,-1,0,2,3,5,7,8,10,11,13,14,16,17,19,20,22,23,25,26,28,29,31,32,34,35,37,38,40,41,43,44,46,47,49,50,300},
{-67,-65,-64,-62,-61,-59,-58,-56,-55,-53,-52,-51,-49,-48,-46,-45,-43,-42,-40,-39,-37,-36,-34,-33,-31,-30,-29,-27,-26,-24,-23,-21,-20,-18,-17,-15,-14,-13,-11,-10,-8,-7,-5,-4,-2,-1,1,2,3,5,6,8,9,11,12,14,15,16,18,19,21,22,24,25,27,28,30,31,32,34,35,37,38,40,41,42,44,45,47,48,300},
{-64,-62,-61,-59,-58,-57,-55,-54,-52,-51,-50,-48,-47,-45,-44,-43,-41,-40,-38,-37,-36,-34,-33,-31,-30,-29,-27,-26,-24,-23,-22,-20,-19,-17,-16,-15,-13,-12,-10,-9,-8,-6,-5,-4,-2,-1,1,2,3,5,6,8,9,10,12,13,15,16,17,19,20,21,23,24,26,27,28,30,31,33,34,35,37,38,39,41,42,44,45,46,300},
{-61,-59,-58,-57,-55,-54,-53,-51,-50,-49,-47,-46,-45,-43,-42,-41,-39,-38,-37,-35,-34,-33,-31,-30,-29,-27,-26,-25,-23,-22,-21,-19,-18,-17,-15,-14,-13,-11,-10,-9,-7,-6,-5,-3,-2,-1,1,2,3,5,6,7,9,10,11,13,14,15,17,18,19,21,22,23,25,26,27,29,30,31,33,34,35,37,38,39,41,42,43,45,300},
{-58,-57,-56,-54,-53,-52,-51,-49,-48,-47,-45,-44,-43,-42,-40,-39,-38,-36,-35,-34,-33,-31,-30,-29,-27,-26,-25,-24,-22,-21,-20,-18,-17,-16,-15,-13,-12,-11,-9,-8,-7,-6,-4,-3,-2,0,1,2,3,5,6,7,9,10,11,12,14,15,16,17,19,20,21,23,24,25,26,28,29,30,31,33,34,35,37,38,39,40,42,43,300},
{-56,-55,-53,-52,-51,-50,-48,-47,-46,-45,-44,-42,-41,-40,-39,-37,-36,-35,-34,-32,-31,-30,-29,-27,-26,-25,-24,-22,-21,-20,-19,-18,-16,-15,-14,-13,-11,-10,-9,-8,-6,-5,-4,-3,-2,0,1,2,3,5,6,7,8,10,11,12,13,14,16,17,18,19,21,22,23,24,25,27,28,29,30,32,33,34,35,36,38,39,40,41,300},
{-54,-53,-51,-50,-49,-48,-47,-45,-44,-43,-42,-41,-39,-38,-37,-36,-35,-33,-32,-31,-30,-29,-27,-26,-25,-24,-23,-22,-20,-19,-18,-17,-16,-14,-13,-12,-11,-10,-8,-7,-6,-5,-4,-3,-1,0,1,2,3,5,6,7,8,9,10,12,13,14,15,16,18,19,20,21,22,23,25,26,27,28,29,31,32,33,34,35,36,38,39,40,300},
{-52,-51,-49,-48,-47,-46,-45,-44,-42,-41,-40,-39,-38,-37,-36,-34,-33,-32,-31,-30,-29,-28,-26,-25,-24,-23,-22,-21,-20,-18,-17,-16,-15,-14,-13,-12,-10,-9,-8,-7,-6,-5,-3,-2,-1,0,1,2,3,5,6,7,8,9,10,11,12,14,15,16,17,18,19,20,22,23,24,25,26,27,28,30,31,32,33,34,35,36,38,39,300},
{-50,-49,-48,-46,-45,-44,-43,-42,-41,-40,-39,-38,-36,-35,-34,-33,-32,-31,-30,-29,-28,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-15,-14,-13,-12,-11,-10,-9,-8,-7,-5,-4,-3,-2,-1,0,1,2,3,4,6,7,8,9,10,11,12,13,14,15,17,18,19,20,21,22,23,24,25,26,28,29,30,31,32,33,34,35,36,37,300},
{-48,-47,-46,-45,-44,-43,-42,-41,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,24,25,26,27,28,29,30,31,32,33,34,35,36,300},
{-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,300},
{-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,300},
{-43,-42,-41,-40,-39,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-3,-2,-1,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,27,28,29,30,31,32,33,300},
{-42,-41,-40,-39,-38,-37,-36,-35,-34,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,31,32,300},
{-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,1,2,3,4,5,6,7,8,9,10,11,12,12,13,14,15,16,17,18,19,20,21,22,22,23,24,25,26,27,28,29,30,31,32,300},
{-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-31,-30,-29,-28,-27,-26,-25,-24,-23,-23,-22,-21,-20,-19,-18,-17,-16,-15,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-6,-5,-4,-3,-2,-1,0,1,2,2,3,4,5,6,7,8,9,10,10,11,12,13,14,15,16,17,17,18,19,20,21,22,23,24,25,25,26,27,28,29,30,31,300},
{-38,-38,-37,-36,-35,-34,-33,-32,-31,-31,-30,-29,-28,-27,-26,-25,-24,-24,-23,-22,-21,-20,-19,-18,-18,-17,-16,-15,-14,-13,-12,-11,-11,-10,-9,-8,-7,-6,-5,-5,-4,-3,-2,-1,0,1,2,2,3,4,5,6,7,8,8,9,10,11,12,13,14,15,15,16,17,18,19,20,21,21,22,23,24,25,26,27,27,28,29,30,300},
{-37,-36,-36,-35,-34,-33,-32,-31,-31,-30,-29,-28,-27,-26,-25,-25,-24,-23,-22,-21,-20,-20,-19,-18,-17,-16,-15,-14,-14,-13,-12,-11,-10,-9,-9,-8,-7,-6,-5,-4,-3,-3,-2,-1,0,1,2,2,3,4,5,6,7,7,8,9,10,11,12,13,13,14,15,16,17,18,18,19,20,21,22,23,23,24,25,26,27,28,28,29,300},
{-36,-35,-35,-34,-33,-32,-31,-30,-30,-29,-28,-27,-26,-26,-25,-24,-23,-22,-21,-21,-20,-19,-18,-17,-16,-16,-15,-14,-13,-12,-12,-11,-10,-9,-8,-7,-7,-6,-5,-4,-3,-2,-2,-1,0,1,2,2,3,4,5,6,7,7,8,9,10,11,11,12,13,14,15,16,16,17,18,19,20,20,21,22,23,24,25,25,26,27,28,29,300},
{-35,-34,-34,-33,-32,-31,-30,-30,-29,-28,-27,-26,-26,-25,-24,-23,-22,-22,-21,-20,-19,-18,-18,-17,-16,-15,-14,-14,-13,-12,-11,-10,-10,-9,-8,-7,-6,-6,-5,-4,-3,-2,-2,-1,0,1,2,2,3,4,5,6,6,7,8,9,10,10,11,12,13,14,14,15,16,17,18,18,19,20,21,22,22,23,24,25,26,26,27,28,300},
{-34,-34,-33,-32,-31,-30,-30,-29,-28,-27,-26,-26,-25,-24,-23,-23,-22,-21,-20,-19,-19,-18,-17,-16,-16,-15,-14,-13,-12,-12,-11,-10,-9,-8,-8,-7,-6,-5,-5,-4,-3,-2,-1,-1,0,1,2,2,3,4,5,6,6,7,8,9,10,10,11,12,13,13,14,15,16,17,17,18,19,20,20,21,22,23,24,24,25,26,27,27,300},
{-33,-33,-32,-31,-30,-30,-29,-28,-27,-27,-26,-25,-24,-23,-23,-22,-21,-20,-20,-19,-18,-17,-17,-16,-15,-14,-14,-13,-12,-11,-10,-10,-9,-8,-7,-7,-6,-5,-4,-4,-3,-2,-1,-1,0,1,2,2,3,4,5,6,6,7,8,9,9,10,11,12,12,13,14,15,15,16,17,18,18,19,20,21,22,22,23,24,25,25,26,27,300},
{-33,-32,-31,-30,-30,-29,-28,-27,-27,-26,-25,-24,-24,-23,-22,-21,-21,-20,-19,-18,-18,-17,-16,-15,-15,-14,-13,-12,-12,-11,-10,-9,-9,-8,-7,-6,-6,-5,-4,-3,-3,-2,-1,0,0,1,2,3,3,4,5,5,6,7,8,8,9,10,11,11,12,13,14,14,15,16,17,17,18,19,20,20,21,22,23,23,24,25,26,26,300},
{-32,-31,-30,-30,-29,-28,-27,-27,-26,-25,-24,-24,-23,-22,-22,-21,-20,-19,-19,-18,-17,-16,-16,-15,-14,-14,-13,-12,-11,-11,-10,-9,-8,-8,-7,-6,-5,-5,-4,-3,-3,-2,-1,0,0,1,2,3,3,4,5,5,6,7,8,8,9,10,11,11,12,13,13,14,15,16,16,17,18,19,19,20,21,21,22,23,24,24,25,26,300},
{-31,-30,-30,-29,-28,-27,-27,-26,-25,-25,-24,-23,-22,-22,-21,-20,-20,-19,-18,-17,-17,-16,-15,-15,-14,-13,-12,-12,-11,-10,-10,-9,-8,-7,-7,-6,-5,-5,-4,-3,-2,-2,-1,0,0,1,2,3,3,4,5,5,6,7,8,8,9,10,10,11,12,12,13,14,15,15,16,17,17,18,19,20,20,21,22,22,23,24,25,25,300},
{-30,-30,-29,-28,-27,-27,-26,-25,-25,-24,-23,-23,-22,-21,-20,-20,-19,-18,-18,-17,-16,-16,-15,-14,-13,-13,-12,-11,-11,-10,-9,-9,-8,-7,-7,-6,-5,-4,-4,-3,-2,-2,-1,0,0,1,2,3,3,4,5,5,6,7,7,8,9,10,10,11,12,12,13,14,14,15,16,16,17,18,19,19,20,21,21,22,23,23,24,25,300},
{-30,-29,-28,-28,-27,-26,-25,-25,-24,-23,-23,-22,-21,-21,-20,-19,-19,-18,-17,-17,-16,-15,-15,-14,-13,-12,-12,-11,-10,-10,-9,-8,-8,-7,-6,-6,-5,-4,-4,-3,-2,-2,-1,0,1,1,2,3,3,4,5,5,6,7,7,8,9,9,10,11,11,12,13,13,14,15,15,16,17,18,18,19,20,20,21,22,22,23,24,24,300},
{-29,-28,-28,-27,-26,-26,-25,-24,-24,-23,-22,-22,-21,-20,-19,-19,-18,-17,-17,-16,-15,-15,-14,-13,-13,-12,-11,-11,-10,-9,-9,-8,-7,-7,-6,-5,-5,-4,-3,-3,-2,-1,-1,0,1,1,2,3,3,4,5,5,6,7,7,8,9,9,10,11,11,12,13,13,14,15,15,16,17,17,18,19,19,20,21,21,22,23,23,24,300},
{-28,-28,-27,-26,-26,-25,-24,-24,-23,-22,-22,-21,-20,-20,-19,-18,-18,-17,-16,-16,-15,-14,-14,-13,-12,-12,-11,-11,-10,-9,-9,-8,-7,-7,-6,-5,-5,-4,-3,-3,-2,-1,-1,0,1,1,2,3,3,4,5,5,6,6,7,8,8,9,10,10,11,12,12,13,14,14,15,16,16,17,18,18,19,20,20,21,22,22,23,23,300},
{-28,-27,-26,-26,-25,-24,-24,-23,-22,-22,-21,-21,-20,-19,-19,-18,-17,-17,-16,-15,-15,-14,-13,-13,-12,-12,-11,-10,-10,-9,-8,-8,-7,-6,-6,-5,-4,-4,-3,-3,-2,-1,-1,0,1,1,2,3,3,4,5,5,6,6,7,8,8,9,10,10,11,12,12,13,13,14,15,15,16,17,17,18,19,19,20,20,21,22,22,23,300},
{-27,-26,-26,-25,-24,-24,-23,-23,-22,-21,-21,-20,-19,-19,-18,-18,-17,-16,-16,-15,-14,-14,-13,-13,-12,-11,-11,-10,-9,-9,-8,-7,-7,-6,-6,-5,-4,-4,-3,-2,-2,-1,-1,0,1,1,2,3,3,4,4,5,6,6,7,8,8,9,9,10,11,11,12,13,13,14,15,15,16,16,17,18,18,19,20,20,21,21,22,23,300},
{-26,-26,-25,-25,-24,-23,-23,-22,-21,-21,-20,-20,-19,-18,-18,-17,-17,-16,-15,-15,-14,-13,-13,-12,-12,-11,-10,-10,-9,-8,-8,-7,-7,-6,-5,-5,-4,-4,-3,-2,-2,-1,0,0,1,1,2,3,3,4,4,5,6,6,7,8,8,9,9,10,11,11,12,12,13,14,14,15,16,16,17,17,18,19,19,20,20,21,22,22,300},
{-26,-25,-25,-24,-23,-23,-22,-22,-21,-20,-20,-19,-19,-18,-17,-17,-16,-16,-15,-14,-14,-13,-13,-12,-11,-11,-10,-9,-9,-8,-8,-7,-6,-6,-5,-5,-4,-3,-3,-2,-2,-1,0,0,1,1,2,3,3,4,4,5,6,6,7,7,8,9,9,10,10,11,12,12,13,13,14,15,15,16,16,17,18,18,19,20,20,21,21,22,300},
{-25,-25,-24,-24,-23,-22,-22,-21,-21,-20,-19,-19,-18,-18,-17,-16,-16,-15,-15,-14,-13,-13,-12,-12,-11,-10,-10,-9,-9,-8,-7,-7,-6,-6,-5,-5,-4,-3,-3,-2,-2,-1,0,0,1,1,2,3,3,4,4,5,6,6,7,7,8,9,9,10,10,11,12,12,13,13,14,14,15,16,16,17,17,18,19,19,20,20,21,22,300},
{-25,-24,-24,-23,-22,-22,-21,-21,-20,-20,-19,-18,-18,-17,-17,-16,-15,-15,-14,-14,-13,-13,-12,-11,-11,-10,-10,-9,-8,-8,-7,-7,-6,-6,-5,-4,-4,-3,-3,-2,-1,-1,0,0,1,1,2,3,3,4,4,5,6,6,7,7,8,8,9,10,10,11,11,12,13,13,14,14,15,15,16,17,17,18,18,19,19,20,21,21,300},
{-24,-24,-23,-23,-22,-21,-21,-20,-20,-19,-19,-18,-17,-17,-16,-16,-15,-15,-14,-13,-13,-12,-12,-11,-11,-10,-9,-9,-8,-8,-7,-7,-6,-5,-5,-4,-4,-3,-3,-2,-1,-1,0,0,1,1,2,3,3,4,4,5,5,6,7,7,8,8,9,9,10,11,11,12,12,13,13,14,15,15,16,16,17,17,18,19,19,20,20,21,300},
{-24,-23,-23,-22,-22,-21,-20,-20,-19,-19,-18,-18,-17,-17,-16,-15,-15,-14,-14,-13,-13,-12,-11,-11,-10,-10,-9,-9,-8,-7,-7,-6,-6,-5,-5,-4,-4,-3,-2,-2,-1,-1,0,0,1,2,2,3,3,4,4,5,5,6,7,7,8,8,9,9,10,11,11,12,12,13,13,14,14,15,16,16,17,17,18,18,19,19,20,21,300},
{-23,-23,-22,-22,-21,-21,-20,-19,-19,-18,-18,-17,-17,-16,-16,-15,-15,-14,-13,-13,-12,-12,-11,-11,-10,-10,-9,-8,-8,-7,-7,-6,-6,-5,-5,-4,-3,-3,-2,-2,-1,-1,0,0,1,2,2,3,3,4,4,5,5,6,7,7,8,8,9,9,10,10,11,11,12,13,13,14,14,15,15,16,16,17,18,18,19,19,20,20,300},
{-23,-22,-22,-21,-21,-20,-20,-19,-19,-18,-17,-17,-16,-16,-15,-15,-14,-14,-13,-13,-12,-11,-11,-10,-10,-9,-9,-8,-8,-7,-7,-6,-5,-5,-4,-4,-3,-3,-2,-2,-1,-1,0,0,1,2,2,3,3,4,4,5,5,6,6,7,8,8,9,9,10,10,11,11,12,12,13,14,14,15,15,16,16,17,17,18,18,19,19,20,300},
{-22,-22,-21,-21,-20,-20,-19,-19,-18,-18,-17,-17,-16,-16,-15,-14,-14,-13,-13,-12,-12,-11,-11,-10,-10,-9,-9,-8,-7,-7,-6,-6,-5,-5,-4,-4,-3,-3,-2,-2,-1,-1,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,9,9,10,10,11,11,12,12,13,13,14,14,15,15,16,17,17,18,18,19,19,20,300},
{-22,-22,-21,-20,-20,-19,-19,-18,-18,-17,-17,-16,-16,-15,-15,-14,-14,-13,-13,-12,-12,-11,-10,-10,-9,-9,-8,-8,-7,-7,-6,-6,-5,-5,-4,-4,-3,-3,-2,-2,-1,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,19,19,300},
{-22,-21,-21,-20,-20,-19,-19,-18,-18,-17,-16,-16,-15,-15,-14,-14,-13,-13,-12,-12,-11,-11,-10,-10,-9,-9,-8,-8,-7,-7,-6,-6,-5,-5,-4,-4,-3,-2,-2,-1,-1,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,14,14,15,15,16,16,17,17,18,18,19,19,300},
{-21,-21,-20,-20,-19,-19,-18,-18,-17,-17,-16,-16,-15,-15,-14,-14,-13,-13,-12,-12,-11,-11,-10,-10,-9,-9,-8,-8,-7,-6,-6,-5,-5,-4,-4,-3,-3,-2,-2,-1,-1,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,19,300},
{-21,-20,-20,-19,-19,-18,-18,-17,-17,-16,-16,-15,-15,-14,-14,-13,-13,-12,-12,-11,-11,-10,-10,-9,-9,-8,-8,-7,-7,-6,-6,-5,-5,-4,-4,-3,-3,-2,-2,-1,-1,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,19,300},
{-21,-20,-20,-19,-19,-18,-18,-17,-17,-16,-16,-15,-15,-14,-14,-13,-13,-12,-12,-11,-11,-10,-10,-9,-9,-8,-8,-7,-7,-6,-6,-5,-5,-4,-4,-3,-3,-2,-2,-1,-1,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13,14,14,15,15,15,16,16,17,17,18,18,300},
{-20,-20,-19,-19,-18,-18,-17,-17,-16,-16,-15,-15,-14,-14,-13,-13,-12,-12,-11,-11,-10,-10,-9,-9,-8,-8,-8,-7,-7,-6,-6,-5,-5,-4,-4,-3,-3,-2,-2,-1,-1,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9,10,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,300},
{-20,-19,-19,-18,-18,-17,-17,-16,-16,-16,-15,-15,-14,-14,-13,-13,-12,-12,-11,-11,-10,-10,-9,-9,-8,-8,-7,-7,-6,-6,-5,-5,-4,-4,-4,-3,-3,-2,-2,-1,-1,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,7,8,8,9,9,10,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,300},
{-19,-19,-19,-18,-18,-17,-17,-16,-16,-15,-15,-14,-14,-13,-13,-12,-12,-11,-11,-11,-10,-10,-9,-9,-8,-8,-7,-7,-6,-6,-5,-5,-4,-4,-3,-3,-2,-2,-2,-1,-1,0,0,1,1,2,2,3,3,4,4,5,5,6,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13,14,14,14,15,15,16,16,17,17,18,300},
        };
        int16  RectifyY[60][81]=
        {
{284,284,284,284,285,285,285,285,285,285,285,285,285,285,285,285,285,285,285,285,285,285,285,285,285,285,285,286,286,286,286,286,286,286,286,286,286,286,286,286,287,287,287,287,287,287,287,287,287,287,288,288,288,288,288,288,288,288,289,289,289,289,289,289,289,289,290,290,290,290,290,290,290,291,291,291,291,291,291,292,300},
{255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,256,256,256,256,256,256,256,256,256,256,256,256,256,256,256,257,257,257,257,257,257,257,257,257,257,257,257,258,258,258,258,258,258,258,258,258,259,259,259,259,259,259,259,259,259,260,260,260,260,260,260,260,261,261,261,261,261,261,261,262,262,262,300},
{231,231,231,231,231,231,231,231,231,231,231,231,231,231,231,231,231,231,231,231,231,232,232,232,232,232,232,232,232,232,232,232,232,232,232,233,233,233,233,233,233,233,233,233,233,233,233,234,234,234,234,234,234,234,234,234,235,235,235,235,235,235,235,235,235,236,236,236,236,236,236,236,237,237,237,237,237,237,237,238,300},
{211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,212,212,212,212,212,212,212,212,212,212,212,212,212,212,213,213,213,213,213,213,213,213,213,213,213,213,214,214,214,214,214,214,214,214,214,215,215,215,215,215,215,215,215,215,216,216,216,216,216,216,216,216,217,217,217,217,217,300},
{194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,195,195,195,195,195,195,195,195,195,195,195,195,195,195,196,196,196,196,196,196,196,196,196,196,196,196,197,197,197,197,197,197,197,197,197,197,198,198,198,198,198,198,198,198,198,199,199,199,199,199,199,199,199,200,200,200,300},
{179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,181,181,181,181,181,181,181,181,181,181,181,181,181,182,182,182,182,182,182,182,182,182,182,182,183,183,183,183,183,183,183,183,183,184,184,184,184,184,184,184,184,184,185,185,185,185,300},
{166,166,166,166,166,166,166,166,166,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,168,168,168,168,168,168,168,168,168,168,168,168,168,168,169,169,169,169,169,169,169,169,169,169,169,169,170,170,170,170,170,170,170,170,170,170,170,171,171,171,171,171,171,171,171,171,172,172,172,172,172,300},
{155,155,155,155,155,155,155,155,155,155,155,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,157,157,157,157,157,157,157,157,157,157,157,157,157,157,158,158,158,158,158,158,158,158,158,158,158,158,158,159,159,159,159,159,159,159,159,159,159,160,160,160,160,160,160,160,160,160,160,161,161,300},
{145,145,145,145,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,148,148,148,148,148,148,148,148,148,148,148,148,148,149,149,149,149,149,149,149,149,149,149,149,149,150,150,150,150,150,150,150,150,150,150,151,151,300},
{137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,139,139,139,139,139,139,139,139,139,139,139,139,139,139,140,140,140,140,140,140,140,140,140,140,140,140,141,141,141,141,141,141,141,141,141,141,142,142,142,142,300},
{129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,132,132,132,132,132,132,132,132,132,132,132,132,132,133,133,133,133,133,133,133,133,133,133,133,134,134,134,134,300},
{122,122,122,122,122,122,122,122,122,122,122,122,122,122,122,122,122,122,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,124,124,124,124,124,124,124,124,124,124,124,124,124,124,124,124,125,125,125,125,125,125,125,125,125,125,125,125,125,126,126,126,126,126,126,126,126,126,126,126,127,127,300},
{116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,119,119,119,119,119,119,119,119,119,119,119,119,120,120,120,120,120,120,120,120,300},
{110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,113,113,113,113,113,113,113,113,113,113,113,113,113,114,114,114,114,114,114,114,114,114,300},
{105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,106,106,106,106,106,106,106,106,106,106,106,106,106,106,106,106,106,106,106,106,107,107,107,107,107,107,107,107,107,107,107,107,107,107,107,107,108,108,108,108,108,108,108,108,108,108,108,108,108,109,109,109,109,300},
{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,103,103,103,103,103,103,103,103,103,103,103,103,103,103,104,104,104,104,104,300},
{95,95,95,95,95,95,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,99,99,99,99,99,99,99,99,99,99,99,300},
{91,91,91,91,91,91,91,91,91,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,95,95,95,95,95,95,95,95,300},
{88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,91,91,91,91,91,91,91,91,91,300},
{84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,86,86,86,86,86,86,86,86,86,86,86,86,86,86,86,86,86,86,87,87,87,87,87,87,87,87,87,87,87,87,87,87,87,87,300},
{81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,82,82,82,82,82,82,82,82,82,82,82,82,82,82,82,82,82,82,82,82,82,82,82,83,83,83,83,83,83,83,83,83,83,83,83,83,83,83,83,83,83,84,84,84,84,84,84,84,84,84,84,300},
{78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,81,81,81,81,81,81,81,300},
{75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,78,78,78,78,78,78,78,78,300},
{72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,75,75,75,75,75,75,75,75,75,75,75,75,300},
{70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,73,73,300},
{67,67,67,67,67,67,67,67,67,67,67,67,67,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,70,70,70,70,70,70,70,70,70,70,70,70,300},
{65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,68,68,68,68,68,68,300},
{63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,66,66,300},
{61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,64,300},
{59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,62,62,300},
{57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,60,60,60,60,60,300},
{56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,58,58,58,58,58,58,58,58,58,58,300},
{54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,300},
{52,52,52,52,52,52,52,52,52,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,55,55,55,55,300},
{51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,300},
{50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,52,52,52,52,300},
{48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,300},
{47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,49,49,49,49,49,49,49,49,49,49,49,300},
{46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,48,48,48,48,300},
{44,44,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,300},
{43,43,43,43,43,43,43,43,43,43,43,43,43,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,300},
{42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,300},
{41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,300},
{40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,300},
{39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,300},
{38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,300},
{37,37,37,37,37,37,37,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,300},
{37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,300},
{36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,300},
{35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,300},
{34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,36,36,36,36,36,300},
{33,33,33,33,33,33,33,33,33,33,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,35,35,35,35,35,35,35,35,35,35,35,35,35,300},
{33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,300},
{32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,300},
{31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,33,33,33,33,33,33,300},
{31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,300},
{30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,300},
{29,29,29,29,29,29,29,29,29,29,29,29,29,29,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,31,31,31,31,31,31,31,300},
{29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,300},
{28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,300},
};

int16  RectifyZX[60][2];
int16  RectifyYX[60][2];

float InvSqrt (float x)
{
float xhalf = 0.5f*x;
int i = *(int *)&x;
i = 0x5f3759df - (i >> 1); // 计算第一个近似根
x = *(float *)&i;
x = x*(1.5f - xhalf*x*x); // 牛顿迭代法
return x;
}

float CurvatureABC (int16 x1 ,int16 y1 ,int16 x2 ,int16 y2 ,int16 x3 ,int16 y3)
{
  float distance1, distance2, distance3, circumference;
  float dividend, divisor;
//  distance1 = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
//  distance2 = sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1));
//  distance3 = sqrt((x3-x2)*(x3-x2)+(y3-y2)*(y3-y2));
  distance1 = 1.0f/InvSqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1)));
  distance2 = 1.0f/InvSqrt(((x3-x1)*(x3-x1))+((y3-y1)*(y3-y1)));
  distance3 = 1.0f/InvSqrt(((x3-x2)*(x3-x2))+((y3-y2)*(y3-y2)));
  circumference = distance1+distance2+distance3;

  dividend = distance1*distance2*distance3;
  divisor = circumference*(circumference-distance1)*(circumference-distance2)*(circumference-distance3);

  return dividend/4*InvSqrt(divisor);
}
//a*b*c/(4*sqrt(p*(p-a)*(p-b)*(p-c)))