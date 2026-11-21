from num2words import num2words


input="29.12.2026"
output="Der nächste Verfügbare Termin ist der "

for number in input.split("."):
    output += num2words(number, lang='de', to='ordinal')
    output += " "

print(output)

def make_time_string_readable(string: str):
        [date, time] = string.split(";")
        [day, month, year] = date.split(".")
        [hour, minute] = time.split(":")
        
        return f"{_num_to_words(day)} {_num_to_words(month)} um {_num_to_words(hour, to="cardinal")} Uhr {_num_to_words(minute, to="cardinal") if minute != "00" else ""}"
        
def _num_to_words(num, to="ordinal"):
    return num2words(num, lang="de", to=to)

print(make_time_string_readable("29.12.2026;11:00"))
