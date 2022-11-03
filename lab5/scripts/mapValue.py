def mapValue(value,oldMin,oldMax,newMin,newMax):
    oldRange = (oldMax - oldMin)  
    newRange = (newMax - newMin)  
    newValue = (((value - oldMin) * newRange) / oldRange) + newMin
    return newValue