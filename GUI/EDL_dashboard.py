#!/usr/bin/env python
# coding: utf-8

# In[1]:
import tkinter as tk
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from tkinter import ttk
from dash.dependencies import Input, Output
import pandas as pd
    # Read data from CSV file
    #df = pd.read_csv('sensordata.csv')
import influxdb_client
from influxdb_client.client.write_api import SYNCHRONOUS


bucket = "SHM_EDL"
org = "EDL Team"
token = "yJMTaGtAVI5PeBlZi8nO0dcarOJmjzrng8fjmzOVUMbDgARnUKhRU3rJcY_6KqVuHGyOcheVfmTIIH3b-8ncUw=="
url="https://us-east-1-1.aws.cloud2.influxdata.com"

client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)
a=str('-1h')
# Query script
query_api = client.query_api()
query = 'from(bucket:"SHM_EDL")\
|> range(start: '+a+')\
|> filter(fn:(r) => r._measurement == "Temp_Hum_Acc")'
result = query_api.query(org=org, query=query)
results = []
df=pd.DataFrame(columns=['acc_x','acc_y','acc_z','temp_iis','time'])
acc_x=[]
acc_y=[]
acc_z=[]
temp_iis=[]
timestamp=[]
time_sys=[]
time=[]

for table in result:
    #print(table)
    for record in table.records:
        #print(record.get_field())
        if(record.get_field()=='acc_x'):
            a=record.get_value()
            #print(a)
            a = a.split(',')[:-1]
            #print(len(a))
            a = [float(val) for val in a]
            acc_x=acc_x+a
        
        if(record.get_field()=='acc_y'):
            a=record.get_value()
            a = a.split(',')[:-1]
            a = [-float(val) for val in a]
            acc_z=acc_z+a
        if(record.get_field()=='acc_z'):
            a=record.get_value()
            a = a.split(',')[:-1]
            a = [float(val) for val in a]
            acc_y=acc_y+a           
        if(record.get_field()=='temp_iis'):
            temp_iis.append((float(record.get_value()),record.get_time()))
        if(record.get_field()=='timestamp'):
            #print(record.get_value())
            a=record.get_value()
            a = a.split(',')[:-1]
            last=a[-1]
            a = [float(val) for val in a]
            last=a[-1]
            time_system = [record.get_time()] * len(a)
            a=[x - last for x in a]
            timestamp=timestamp+a
            time_sys=time_sys+time_system
temp_iisdf = pd.DataFrame(temp_iis, columns=['temp_iis', 'time'])
temp_iisdf.set_index('time', inplace=True)
temp_iisdf['time']=pd.to_datetime(temp_iisdf.index , utc=True)+pd.Timedelta(hours=5, minutes=30)                 
#print(acc_x)  
data = {'acc_x': acc_x, 'acc_y': acc_y, 'acc_z': acc_z,'time_sys':time_sys,'timestamp':timestamp}
df = pd.DataFrame(data)
import datetime
df['time_sys']=pd.to_datetime(df['time_sys'] , utc=True)+pd.Timedelta(hours=5, minutes=30)
df['timestamp']=1000000*df['timestamp']
df['time']=[dt + datetime.timedelta(microseconds=us) 
                              for dt, us in zip(df['time_sys'], df['timestamp'])]
#df['time']=df.index
entry_start_time="2024-04-13 05:30:00"
entry_stop_time="2024-04-30 15:30:00"
# Filter data based on start and stop time
start_time = pd.to_datetime(entry_start_time, utc=True)
stop_time = pd.to_datetime(entry_stop_time ,utc=True)

df_filtered = df[(df['time'] >= start_time) & (df['time'] <= stop_time)]
temp_iisdf  =temp_iisdf[(temp_iisdf['time'] >= start_time) & (temp_iisdf['time'] <= stop_time)]


fft_x = np.abs(np.fft.fft(df_filtered['acc_x']))
freq_x = np.fft.fftfreq(len(df_filtered['acc_x']), 10)# Plot data on each subplot
fft_y = np.abs(np.fft.fft(df_filtered['acc_y']))
freq_y = np.fft.fftfreq(len(df_filtered['acc_y']), 10)# Plot data on each subplot
fft_z = np.abs(np.fft.fft(df_filtered['acc_z']))
freq_z = np.fft.fftfreq(len(df_filtered['acc_z']), 10)# Plot data on each subplot

mean_temp=np.mean(temp_iisdf['temp_iis'])
std_temp=np.mean(temp_iisdf['temp_iis'])
mean_x=np.mean(df_filtered['acc_x'])
std_x=np.std(df_filtered['acc_x'])
mean_y=np.mean(df_filtered['acc_y'])
std_y=np.std(df_filtered['acc_y'])
mean_z=np.mean(df_filtered['acc_z'])
std_z=np.std(df_filtered['acc_z'])

# In[52]:


import dash
#import dash_core_components as dcc
from dash import dcc
#import dash_html_components as html
from dash import html
import plotly.graph_objs as go
from textwrap import dedent


# In[53]:


import io
import xlsxwriter
import flask
from flask import send_file


# In[54]:


#import dash_table_experiments as dt
from textwrap import dedent


# In[55]:



trace1 = go.Scatter(x=df_filtered['time'], y= df_filtered['acc_x'],mode="lines+markers",fillcolor="red",name="acc_x")
trace2 = go.Scatter(x=df_filtered['time'], y= df_filtered['acc_y'],mode="lines+markers",fillcolor="blue",name="acc_y")
trace3 = go.Scatter(x=df_filtered['time'], y= df_filtered['acc_z'],mode="lines+markers",fillcolor="green",name="acc_z")
trace4 = go.Scatter(x=freq_x, y= fft_x,mode="lines+markers",fillcolor="cyan",name="FFT_X")
trace5 = go.Scatter(x=freq_y, y= fft_y,mode="lines+markers",fillcolor="brown",name="FFT_Y")
trace6 = go.Scatter(x=freq_z, y= fft_z,mode="lines+markers",fillcolor="pink",name="FFT_Z")
trace7 = go.Scatter(x=temp_iisdf['time'], y= temp_iisdf['temp_iis'],mode="lines+markers",fillcolor="pink",name="FFT_Z")


import os
external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
app = dash.Dash(__name__, external_stylesheets=external_stylesheets)
server = app.server
app_name = 'My_Dash'
# Boostrap CSS.
#app.css.append_css({'external_url': 'https://cdn.rawgit.com/plotly/dash-app-stylesheets/2d266c578d2a6e8850ebce48fdb52759b2aef506/stylesheet-oil-and-gas.css'})  # noqa: E501

layout = dict(
    autosize=True,
    height=500,
    font=dict(color="#191A1A"),
    titlefont=dict(color="#191A1A", size='14'),
    margin=dict(
        l=35,
        r=35,
        b=35,
        t=45
    )
)


app.layout = html.Div([
    html.H1("Wireless Vibration Sensing for Structural Health Monitoring", style={"textAlign": "center"}),
    
    html.Div(  
        dcc.Markdown('''From this dashboard we can monitor the vibration data in 3 axes X, Y, and Z and the temperature.
                     Team MON-01: Aditya Bhangale, Ayush Joshi, Harsh Shah, Soham Inamdar.
        '''), style={'color':'black','textAlign':'left'}
    ),
    
    html.Div([
        # Dropdown for selecting time interval
        dcc.Dropdown(
            id='time-interval-dropdown',
            options=[
                {'label': 'Past 1m', 'value': '-1m'},
                {'label': 'Past 2m', 'value': '-2m'},
                {'label': 'Past 3m', 'value': '-3m'},
                {'label': 'Past 5m', 'value': '-5m'},
                {'label': 'Past 10m', 'value': '-10m'},
                {'label': 'Past 15m', 'value': '-15m'},
                {'label': 'Past 30m', 'value': '-30m'},
                {'label': 'Past 1hr', 'value': '-1h'},
                {'label': 'Past 3hrs', 'value': '-3h'},
                {'label': 'Past 6hrs', 'value': '-6h'},
                {'label': 'Past 12hrs', 'value': '-12h'},
                {'label': 'Past 24hrs', 'value': '-24h'},
            ],
            value='10m'  # Default value
        )
    ], className="six columns", style={"width": "40%", "float": "right"}),
    
    # Graphs
    html.Div([
        # Graph 1
        html.Div([
            dcc.Graph(
                id='my-graph2a',
                figure={
                    'data': [trace1],
                    'layout': {
                        'title': 'Acceleration Data : X-axis',
                        'xaxis': {'title': f'Mean: {mean_y:.2f}'+f'Std. Dev: {std_y:.2f}'+'      Time -->'},
                        'yaxis': {'title': 'Acceleration(mg) -->'},
                        'line': {'color': 'blue'},
                        'caption': f'Mean: {mean_x:.2f}'+f'Std. Dev: {std_x:.2f}'
                    }
                },
                className="six columns"
            )
        ]),
        
        # Graph 2
        html.Div([
            dcc.Graph(
                id='my-graph2b',
                figure={
                    'data': [trace2],
                    'layout': {
                        'title': 'Acceleration Data : Y-axis',
                        'xaxis': {'title':f'Mean: {mean_y:.2f}'+f'Std. Dev: {std_y:.2f}'+ '      Time -->'},
                        'yaxis': {'title': 'Acceleration(mg) -->'},
                        'line': {'color': 'red'},
                        'caption': f'Mean: {mean_y:.2f}'+f'Std. Dev: {std_y:.2f}'
                    }
                },
                className="six columns"
            )
        ]),
        
        # Graph 3
        html.Div([
            dcc.Graph(
                id='my-graph2c',
                figure={
                    'data': [trace3],
                    'layout': {
                        'title': 'Acceleration Data : Z-axis',
                        'xaxis': {'title': f'Mean: {mean_z:.2f}'+f'Std. Dev: {std_z:.2f}'+'      Time -->'},
                        'yaxis': {'title': 'Acceleration(mg) -->'},
                        'line': {'color': 'green'},
                        'caption': f'Mean: {mean_z:.2f}'+f'Std. Dev: {std_z:.2f}'
                    }
                },
                className="six columns"
            )
        ]),
        # Graph 4
        html.Div([
            dcc.Graph(
                id='my-graph3a',
                figure={
                    'data': [trace4],
                    'layout': {
                        'title': 'FFT_X',
                        'xaxis': {'title': 'Frequency (Hz) -->'},
                        'yaxis': {'title': 'Magnitude -->'},
                        'line': {'color': 'cyan'}
                    }
                },
                className="six columns"
            )
        ]),
        # Graph 5
        html.Div([
            dcc.Graph(
                id='my-graph3b',
                figure={
                    'data': [trace5],
                    'layout': {
                        'title': 'FFT_Y',
                        'xaxis': {'title': 'Frequency (Hz) -->'},
                        'yaxis': {'title': 'Magnitude -->'},
                        'line': {'color': 'purple'}
                    }
                },
                className="six columns"
            )
        ]),
        # Graph 6
        html.Div([
            dcc.Graph(
                id='my-graph3c',
                figure={
                    'data': [trace6],
                    'layout': {
                        'title': 'FFT_Z',
                        'xaxis': {'title': 'Frequency (Hz) -->'},
                        'yaxis': {'title': 'Magnitude -->'},
                        'line': {'color': 'brown'}
                    }
                },
                className="six columns"
            )
        ]),
        # Graph 7
        html.Div([
            dcc.Graph(
                id='my-graph3d',
                figure={
                    'data': [trace7],
                    'layout': {
                        'title': 'Temperature',
                        'xaxis': {'title': 'Temperature (*C)-->'},
                        'yaxis': {'title': f'Mean: {mean_temp:.2f}'+f'Std. Dev: {std_temp:.2f}'+ '      Time -->'},
                        'line': {'color': 'Yellow'},
                        'caption': f'Mean: {mean_temp:.2f}'+f'Std. Dev: {std_temp:.2f}'
                    }
                },
                className="six columns"
            )
        ]),
    ], className="row")
])

   

    



# Callback to update graphs based on selected time interval
@app.callback(
    [Output('my-graph2a', 'figure'),
     Output('my-graph2b', 'figure'),
     Output('my-graph2c', 'figure'),
     Output('my-graph3a', 'figure'),
     Output('my-graph3b', 'figure'),
     Output('my-graph3c', 'figure'),
     Output('my-graph3d', 'figure')],
    [Input('time-interval-dropdown', 'value')])
def update_graph(time_interval):
    bucket = "SHM_EDL"
    org = "EDL Team"
    token = "yJMTaGtAVI5PeBlZi8nO0dcarOJmjzrng8fjmzOVUMbDgARnUKhRU3rJcY_6KqVuHGyOcheVfmTIIH3b-8ncUw=="
    url="https://us-east-1-1.aws.cloud2.influxdata.com"

    client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)
    a=str(time_interval)
    # Query script
    query_api = client.query_api()
    query = 'from(bucket:"SHM_EDL")\
    |> range(start: '+a+')\
    |> filter(fn:(r) => r._measurement == "Temp_Hum_Acc")'
    result = query_api.query(org=org, query=query)
    results = []
    df=pd.DataFrame(columns=['acc_x','acc_y','acc_z','temp_iis','time'])
    acc_x=[]
    acc_y=[]
    acc_z=[]
    temp_iis=[]
    timestamp=[]
    time_sys=[]
    time=[]

    for table in result:
        #print(table)
        for record in table.records:
            #print(record.get_field())
            if(record.get_field()=='acc_x'):
                a=record.get_value()
                #print(a)
                a = a.split(',')[:-1]
                #print(len(a))
                a = [float(val) for val in a]
                acc_x=acc_x+a
            
            if(record.get_field()=='acc_y'):
                a=record.get_value()
                a = a.split(',')[:-1]
                a = [-float(val) for val in a]
                acc_z=acc_z+a
            if(record.get_field()=='acc_z'):
                a=record.get_value()
                a = a.split(',')[:-1]
                a = [float(val) for val in a]
                acc_y=acc_y+a           
            if(record.get_field()=='temp_iis'):
                temp_iis.append((float(record.get_value()),record.get_time()))
            if(record.get_field()=='timestamp'):
                #print(record.get_value())
                a=record.get_value()
                a = a.split(',')[:-1]
                last=a[-1]
                a = [float(val) for val in a]
                last=a[-1]
                time_system = [record.get_time()] * len(a)
                a=[x - last for x in a]
                timestamp=timestamp+a
                time_sys=time_sys+time_system
    temp_iisdf = pd.DataFrame(temp_iis, columns=['temp_iis', 'time'])
    temp_iisdf.set_index('time', inplace=True)
    temp_iisdf['time']=pd.to_datetime(temp_iisdf.index , utc=True)+pd.Timedelta(hours=5, minutes=30)                 
    #print(acc_x)  
    data = {'acc_x': acc_x, 'acc_y': acc_y, 'acc_z': acc_z,'time_sys':time_sys,'timestamp':timestamp}
    df = pd.DataFrame(data)
    import datetime
    df['time_sys']=pd.to_datetime(df['time_sys'] , utc=True)+pd.Timedelta(hours=5, minutes=30)
    df['timestamp']=1000000*df['timestamp']
    df['time']=[dt + datetime.timedelta(microseconds=us) 
                                for dt, us in zip(df['time_sys'], df['timestamp'])]
    #df['time']=df.index
    entry_start_time="2024-04-13 05:30:00"
    entry_stop_time="2024-04-30 15:30:00"
    # Filter data based on start and stop time
    start_time = pd.to_datetime(entry_start_time, utc=True)
    stop_time = pd.to_datetime(entry_stop_time ,utc=True)

    df_filtered = df[(df['time'] >= start_time) & (df['time'] <= stop_time)]
    temp_iisdf  =temp_iisdf[(temp_iisdf['time'] >= start_time) & (temp_iisdf['time'] <= stop_time)]


    fft_x = np.abs(np.fft.fft(df_filtered['acc_x']))
    freq_x = np.fft.fftfreq(len(df_filtered['acc_x']), 10)# Plot data on each subplot
    fft_y = np.abs(np.fft.fft(df_filtered['acc_y']))
    freq_y = np.fft.fftfreq(len(df_filtered['acc_y']), 10)# Plot data on each subplot
    fft_z = np.abs(np.fft.fft(df_filtered['acc_z']))
    freq_z = np.fft.fftfreq(len(df_filtered['acc_z']), 10)# Plot data on each subplot

    mean_temp=np.mean(temp_iisdf['temp_iis'])
    std_temp=np.mean(temp_iisdf['temp_iis'])
    mean_x=np.mean(df_filtered['acc_x'])
    std_x=np.std(df_filtered['acc_x'])
    mean_y=np.mean(df_filtered['acc_y'])
    std_y=np.std(df_filtered['acc_y'])
    mean_z=np.mean(df_filtered['acc_z'])
    std_z=np.std(df_filtered['acc_z'])
        # Assuming df_filtered, freq_x, freq_y, freq_z, fft_x, fft_y, fft_z are defined elsewhere
    trace1 = go.Scatter(x=df_filtered['time'], y=df_filtered['acc_x'], mode="lines+markers", fillcolor="red", name="acc_x")
    trace2 = go.Scatter(x=df_filtered['time'], y=df_filtered['acc_y'], mode="lines+markers", fillcolor="blue", name="acc_y")
    trace3 = go.Scatter(x=df_filtered['time'], y=df_filtered['acc_z'], mode="lines+markers", fillcolor="green", name="acc_z")
    trace4 = go.Scatter(x=freq_x, y=fft_x, mode="lines+markers", fillcolor="cyan", name="FFT_X")
    trace5 = go.Scatter(x=freq_y, y=fft_y, mode="lines+markers", fillcolor="brown", name="FFT_Y")
    trace6 = go.Scatter(x=freq_z, y=fft_z, mode="lines+markers", fillcolor="pink", name="FFT_Z")
    trace7 = go.Scatter(x=temp_iisdf['time'], y=temp_iisdf['temp_iis'], mode="lines+markers", fillcolor="pink", name="Temp_IIS")

    fig1 = {
        'data': [trace1],
        'layout': {
            'title': 'Acceleration Data : X-axis ',
            'xaxis': {'title': f'Mean: {mean_x:.2f}'+f'Std. Dev: {std_x:.2f}'+ '      Time -->'},
            'yaxis': {'title': 'Acceleration(mg) -->'},
            'line': {'color': 'red'},
            'caption': f'Mean: {mean_x:.2f}'+f'Std. Dev: {std_x:.2f}'
        }
    }
    
    fig2 = {
        'data': [trace2],
        'layout': {
            'title': 'Acceleration Data : Y-axis',
            'xaxis': {'title': f'Mean: {mean_y:.2f}'+f'Std. Dev: {std_y:.2f}'+ '      Time -->'},
            'yaxis': {'title': 'Acceleration(mg) -->'},
            'line': {'color': 'blue'},
            'caption': f'Mean: {mean_y:.2f}'+f'Std. Dev: {std_y:.2f}'
        }
    }
    
    fig3 = {
        'data': [trace3],
        'layout': {
            'title': 'Acceleration Data : Z-axis',
            'xaxis': {'title':  f'Mean: {mean_z:.2f}'+f'Std. Dev: {std_z:.2f}'+ '      Time -->'},
            'yaxis': {'title': 'Acceleration(mg) -->'},
            'line': {'color': 'green'},
            'caption': f'Mean: {mean_y:.2f}'+f'Std. Dev: {std_y:.2f}'
        }
    }
    
    fig4 = {
        'data': [trace4],
        'layout': {
            'title': 'FFT_X',
            'xaxis': {'title': 'Frequency (Hz) -->'},
            'yaxis': {'title': 'Magnitude -->'},
            'line': {'color': 'cyan'}
        }
    }
    
    fig5 = {
        'data': [trace5],
        'layout': {
            'title': 'FFT_Y',
            'xaxis': {'title': 'Frequency (Hz) -->'},
            'yaxis': {'title': 'Magnitude -->'},
            'line': {'color': 'brown'}
        }
    }
    
    fig6 = {
        'data': [trace6],
        'layout': {
            'title': 'FFT_Z',
            'xaxis': {'title': 'Frequency (Hz) -->'},
            'yaxis': {'title': 'Magnitude -->'},
            'line': {'color': 'pink'}
        }
    }
    
    fig7 = {
        'data': [trace7],
        'layout': {
            'title': 'Temperature',
            'xaxis': {'title': 'Temperature (*C)-->'},
            'yaxis': {'title': f'Mean: {mean_temp:.2f}'+f'Std. Dev: {std_temp:.2f}'+'      Time -->'},
            'line': {'color': 'yellow'},
            'caption': f'Mean: {mean_temp:.2f}'+f'Std. Dev: {std_temp:.2f}'
        }
    }

    return fig1, fig2, fig3, fig4, fig5, fig6, fig7
    return figure
    # Update the original graph with the modified figure object
    #original_graph.update(original_fig)




if __name__ == '__main__':
    app.run_server(debug=False)


# In[ ]:




