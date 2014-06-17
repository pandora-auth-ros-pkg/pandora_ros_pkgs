def parseMessage(messagePath):
  fieldList = []

  sep = '#'

  with open(messagePath,'r') as messageFile:
    for line in messageFile.readlines():
      line = line.strip()
      if line and not line.startswith(sep) and not '=' in line :
        field = tuple(line.split(sep, 1)[0].split())
        if not field[0] == 'Header':
          fieldList.append(field)

  return fieldList
