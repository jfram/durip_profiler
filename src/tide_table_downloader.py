import requests
import pandas as pd

if __name__ == "__main__":

    print("Enter station ID: ", end="")
    station = input()

    print("Enter start date in YYYYMMDD format: ", end="")
    start_date = input()
    year = start_date[:4]
    month = start_date[4:6]
    day = start_date[6:]
    print("Verify the entry is correct: " + year + "/" + month + "/" + day)

    print("Enter end date in YYYYMMDD format: ", end="")
    end_date = input()
    year = end_date[:4]
    month = end_date[4:6]
    day = end_date[6:]
    print("Verify the entry is correct: " + year + "/" + month + "/" + day)

    base_url = "https://api.tidesandcurrents.noaa.gov/api/prod/datagetter?"
    param_url = "begin_date="+start_date+"&end_date="+end_date+"&station="+station
    suffix_url = "&product=predictions&datum=MLLW&time_zone=gmt&interval=hilo&units=metric&application=OOI_AScherer&format=csv"
    response = requests.get(base_url + param_url + suffix_url)

    file_path = './tides.txt'
    with open(file_path, 'wb') as out_file:
        out_file.write(response.content)

    df = pd.read_csv(file_path, usecols=['Date Time'], parse_dates={'datetime': [0]})

    df['epoch'] = (df.datetime.astype('int64')/1e9).astype(int)
    df = df.set_index('epoch')
    df = df.drop(columns='datetime')

    df.to_csv(file_path, header=False)
