import re

myaction = '#000P1500T000!'
regex = re.compile("[#!]")
tempList = regex.split(myaction)
print(tempList)

return_str = 'AAA^BBB'
tempList = return_str.split('^')
print(tempList[1])

return_str = 'AAA^BBB'
return_str = return_str.replace('^', '\0')
print(return_str)

return_str = '111:#111!>'
if return_str.find('$') > 0:
    return_str = return_str.split('$')
    return_str = return_str[1]
    return_str = return_str.split('!')
    return_str = return_str[0]
    return_str = '$' +return_str+ '!'
    print(return_str)
elif return_str.find('#') > 0:
    return_str = return_str.split(':')
    return_str = return_str[1]
    return_str = return_str.replace('>','\0')
    print(return_str)
