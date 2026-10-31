from num2words import num2words


input="29.12.2026"
output="Der nächste Verfügbare Termin ist der "

for number in input.split("."):
    output += num2words(number, lang='de', to='ordinal')
    output += " "

print(output)
