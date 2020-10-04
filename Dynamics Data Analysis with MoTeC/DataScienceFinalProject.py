#!/usr/bin/env python
# coding: utf-8

# In[1]:


import io
import urllib, base64
import matplotlib.pyplot as plt
import matplotlib.lines as lines
import seaborn as sns
import statsmodels.api as sm
import pandas as pd
import numpy as np
import csv
import math


# In[2]:


rm_quote = lambda x: float(x.replace('"', '')) #sets up removing the double quotes
with open('specs.csv', mode='r') as infile:
    reader = csv.reader(infile)
    with open('specs_new.csv', mode='w') as outfile:
        writer = csv.writer(outfile)
        specs = {rows[0]:rows[1] for rows in reader}


# In[ ]:


conv = {} # dict for removing quotes 
for i in range(62):
    conv[i]=rm_quote

#pull notes from csv
notes = pd.read_csv('data stuff.csv', sep=None, names=(0,1,2,3,4,5,6,7), skiprows=[0], error_bad_lines=False, nrows = 11, header=None, engine='python')
notes = notes.iloc[:, [0, 1, 2, 4, 5, 6]]

#pulls data from the whatnot    
data = pd.read_csv('data stuff.csv', skiprows=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,15] , low_memory = False, converters=conv)


#pulls column titles and units
units=pd.read_csv('data stuff.csv', skiprows=14, nrows = 1)


# In[4]:


df = pd.DataFrame(data)
cleaned_df = df.loc[:, (df != df.iloc[0]).any()] 
# Make a list of broken sensors
broken_sensors = []
for item in df.columns:
    if item not in cleaned_df.columns:
        broken_sensors.append(item)

# double-check the head to see that the dataframe is cleaned
cleaned_df.head()


# In[5]:


# Converts matplotlib figure 'fig' into a string containing an <img> tag for HTML use using base64 encoding.
def convert_fig_to_html(fig):
    sio = io.BytesIO()
    fig.savefig(sio, format='png')
    sio.seek(0)
    
    string = base64.b64encode(sio.read())
    uri = 'data:image/png;base64,' + urllib.parse.quote(string)
    html = '<img src = "%s"/>' % uri
    return html


# In[6]:


# check specs
print(specs)


# In[7]:


# Use at very end
cleaned_df = cleaned_df[:25000]


# In[8]:


#Zero damper positions

def calibrate_dampers(df):
    is_start = True
    if 'Wheel Speed FL' in df.columns:
        if df.loc[0, "Wheel Speed FL"] != 0:
            is_start = False
    if 'Wheel Speed FR' in df.columns:
        if df.loc[0, "Wheel Speed FR"] != 0:
            is_start = False
    if 'Wheel Speed RL' in df.columns:
        if df.loc[0, "Wheel Speed RL"] != 0:
            is_start = False
    if 'Wheel Speed RR' in df.columns:
        if df.loc[0, "Wheel Speed RR"] != 0:
            is_start = False
    
    if (is_start):
        df["Damper Pos FL"].apply(lambda x: x - df.loc[0, "Damper Pos FL"])
        df["Damper Pos FR"].apply(lambda x: x - df.loc[0, "Damper Pos FR"])
        df["Damper Pos RL"].apply(lambda x: x - df.loc[0, "Damper Pos RL"])
        df["Damper Pos RR"].apply(lambda x: x - df.loc[0, "Damper Pos RR"])
        
calibrate_dampers(cleaned_df)


# In[9]:


# Gating function
def gating(df, col, gate):
    drop = []
    count = 0
    length = len(df.index)
    colind = df.columns.get_loc(col)
    if col in df.columns:
        for row in range(0, length):
            if (abs(df.loc[row, col]) < gate ):
                drop.append(row)
    temp = df.drop(drop, axis=0)
    return temp


# In[10]:


# Generate Velocity Histograms for Dampers
def calculate_damper_velocities(df):
    delta = df.iloc[1, df.columns.get_loc("Time")] - df.iloc[0, df.columns.get_loc("Time")]
    if "Damper Pos FL" in df.columns:
        ## Units: mm/s
        fl_damper_velocity = np.gradient(df.loc[:, "Damper Pos FL"], delta)
    if "Damper Pos FR" in df.columns:
        fr_damper_velocity = np.gradient(df.loc[:, "Damper Pos FR"], delta)
    if "Damper Pos RL" in df.columns:
        rl_damper_velocity = np.gradient(df.loc[:, "Damper Pos RL"], delta)
    if "Damper Pos RR" in df.columns:
        rr_damper_velocity = np.gradient(df.loc[:, "Damper Pos RR"], delta)
    return fl_damper_velocity, fr_damper_velocity, rl_damper_velocity, rr_damper_velocity
        
def generate_damper_histograms(df):
    fl_damper_velocity, fr_damper_velocity, rl_damper_velocity, rr_damper_velocity = calculate_damper_velocities(df)
    # sns.set_style('darkgrid')

    fl_ax = sns.distplot(fl_damper_velocity, kde=False, norm_hist=True, color="purple")
    fl_ax.set(xlabel='Damper Velocity FL (mm/s)', ylabel='Frequency')
    plt.show()
    
    fr_ax = sns.distplot(fr_damper_velocity, kde=False, norm_hist=True, color="red")
    fr_ax.set(xlabel='Damper Velocity FR (mm/s)', ylabel='Frequency')
    plt.show()
    
    rl_ax = sns.distplot(rl_damper_velocity, kde=False, norm_hist=True)
    rl_ax.set(xlabel='Damper Velocity RL (mm/s)', ylabel='Frequency')
    plt.show()
    
    rr_ax = sns.distplot(rr_damper_velocity, kde=False, norm_hist=True,color="green")
    rr_ax.set(xlabel='Damper Velocity RR (mm/s)', ylabel='Frequency')
    plt.show()

    return fl_ax.get_figure(), fr_ax.get_figure(), rl_ax.get_figure(), rr_ax.get_figure()

generate_damper_histograms(cleaned_df)
# generate_damper_histograms(gating(cleaned_df, "G Force Lat", 1))


# In[11]:


# Calculating Roll Angles

# Wheel travel arrays
wt_fl = []
wt_fr = []
wt_rl = []
wt_rr = []


# Roll Degree Arrays
FRD = []
RRD = []

#Roll Gradient Arrays
frollgrad = []
rrollgrad = []
avgrollgrad = []

# New Dataframe
df1 = {}

# Calculate wheel travel which is in mm
# damper pos is in mm
for row in cleaned_df.index:
    val1 = cleaned_df.loc[row, "Damper Pos FL"]/float(specs['Spring IR Front (in/in)'])
    val2 = cleaned_df.loc[row, "Damper Pos FR"]/float(specs['Spring IR Front (in/in)'])
    val3 = cleaned_df.loc[row, "Damper Pos RL"]/float(specs['Spring IR Rear (in/in)'])
    val4 = cleaned_df.loc[row, "Damper Pos RR"]/float(specs['Spring IR Rear (in/in)'])
    
    wt_fl.append(val1)
    wt_fr.append(val2)
    wt_rl.append(val3)
    wt_rr.append(val4)

# Calculate front and rear roll degree
# Need to check units
for row in cleaned_df.index:
    frolldeg = math.atan2(wt_fl[row] - wt_fr[row], float(specs['Track Front (mm)'])) * 180/math.pi
    rrolldeg = math.atan2(wt_rl[row] - wt_rr[row], float(specs['Track Rear (mm)'])) * 180/math.pi
    
    val1 = frolldeg/cleaned_df.loc[row, "G Force Lat"] if (cleaned_df.loc[row, "G Force Lat"] != 0) else None
    val2 = rrolldeg/cleaned_df.loc[row, "G Force Lat"] if (cleaned_df.loc[row, "G Force Lat"] != 0) else None
    avg = 0.5 * (val1 + val2) if (val1 and val2) else None
    
    # Append Roll Degree
    FRD.append(frolldeg)
    RRD.append(rrolldeg)
    
    # Calculate front/rear roll gradient and average roll gradient
    frollgrad.append(val1)
    rrollgrad.append(val2)   
    avgrollgrad.append(avg)
    

# New Dataframe with updated values
df1.update({"Wheel Travel FL": wt_fl})
df1.update({"Wheel Travel FR": wt_fr})
df1.update({"Front Roll Degree": FRD})
df1.update({"Rear Roll Degree": RRD})
# df1.update({"Average Roll Gradient": avgrollgrad})


new_df = pd.DataFrame.from_dict(df1)
result = cleaned_df.join(new_df)


# In[12]:


#Obtaining lists of G forces
Ay = np.array(list(cleaned_df.loc[:, "G Force Lat"]))
Ax = np.array(list(cleaned_df.loc[:, "G Force Long"]))


# In[13]:


# function to plot roll gradients
def plot_line_df(x_name, y_name, df):
    ax = sns.lmplot(x_name, y_name, df)
    return ax.fig

# Finding Front, Rear, and Average Roll Gradient
FRG = {"Ay": Ay, "Front Roll Degree": result.loc[:,"Front Roll Degree"]}
RRG = {"Ay": Ay, "Rear Roll Degree": result.loc[:,"Rear Roll Degree"]}

line_df_frg = pd.DataFrame(FRG)
line_df_rrg = pd.DataFrame(RRG)

# Finding gated versions of roll gradient
temp = gating(result, "G Force Lat", 1)
temp_Ay = np.array(list(temp.loc[:, "G Force Lat"]))

TFRG = {"Ay": temp_Ay, "Front Roll Degree": temp.loc[:,"Front Roll Degree"]}
TRRG = {"Ay": temp_Ay, "Rear Roll Degree": temp.loc[:,"Rear Roll Degree"]}
t_line_df_frg = pd.DataFrame(TFRG)
t_line_df_rrg = pd.DataFrame(TRRG)

plot_line_df("Ay", "Front Roll Degree", line_df_frg)
plot_line_df("Ay", "Rear Roll Degree", line_df_rrg)

plot_line_df("Ay", "Front Roll Degree", t_line_df_frg)
plot_line_df("Ay", "Rear Roll Degree", t_line_df_rrg)


def getSlope(x, y):
    res = sm.OLS(y, x).fit()
    coeff = (res.params).iloc[0]
    return coeff


front_roll_gradient = getSlope(Ay, result.loc[:,"Front Roll Degree"])
rear_roll_gradient = getSlope(Ay, result.loc[:,"Rear Roll Degree"])
average_roll_gradient = 0.5 * (front_roll_gradient + rear_roll_gradient)
print("Front roll gradient: {}".format(front_roll_gradient))
print("Rear roll gradient: {}".format(rear_roll_gradient))
print("Average roll gradient: {}".format(average_roll_gradient))

t_front_roll_gradient = getSlope(temp_Ay, temp.loc[:,"Front Roll Degree"])
t_rear_roll_gradient = getSlope(temp_Ay, temp.loc[:,"Rear Roll Degree"])
t_average_roll_gradient = 0.5 * (t_front_roll_gradient + t_rear_roll_gradient)
print('\n')
print("Gated front roll gradient: {}".format(t_front_roll_gradient))
print("Gated rear roll gradient: {}".format(t_rear_roll_gradient))
print("Gated average roll gradient: {}".format(t_average_roll_gradient))

def roll_gradient_HTML():
    front_roll_gradient = getSlope(Ay, result.loc[:,"Front Roll Degree"])
    rear_roll_gradient = getSlope(Ay, result.loc[:,"Rear Roll Degree"])
    average_roll_gradient = 0.5 * (front_roll_gradient + rear_roll_gradient)
    t_front_roll_gradient = getSlope(temp_Ay, temp.loc[:,"Front Roll Degree"])
    t_rear_roll_gradient = getSlope(temp_Ay, temp.loc[:,"Rear Roll Degree"])
    t_average_roll_gradient = 0.5 * (t_front_roll_gradient + t_rear_roll_gradient)
    string1 = "Front roll gradient: {}<br/>Rear roll gradient: {}<br/>Average roll gradient: {}<br/>".format(front_roll_gradient, rear_roll_gradient, average_roll_gradient)
    string2 = "Gated front roll gradient: {}<br/>Gated rear roll gradient: {}<br/>Gated average roll gradient: {}".format(t_front_roll_gradient, t_rear_roll_gradient, t_average_roll_gradient)
    
    return string1, string2


# In[14]:


# Calculating Lateral Load Transfer

def plot_LLT(Ay, time):
    # lbs * g's
    # LLT uses average of rear and front track widths
    temp = Ay
    avg_track_width = 0.5 * (float(specs["Track Front (mm)"]) + float(specs["Track Rear (mm)"]))
    val = list(map(lambda x: x * float(specs["CG Height (in)"])/(avg_track_width/25.4), temp))
    LLT = {"LLT Coefficients": val, "Time (s)": time}
    line_df_LLT = pd.DataFrame(LLT)
    fig = plot_line_df("Time (s)", "LLT Coefficients", line_df_LLT)
    # print("Maximum Lateral Load Transfer: {}\nMinimum Lateral Load Transfer: {}".format(max(val), min(val)))
    string = "Maximum Lateral Load Transfer: {}<br/>Minimum Lateral Load Transfer: {}<br/>".format(max(val), min(val))
    return fig, string

fig, string = plot_LLT(Ay, cleaned_df["Time"])
print(string)


# In[15]:


# Plotting GG plot
def plot_GG(Ay, Ax):
    fig, ax = plt.subplots()
    ax.scatter(Ay, Ax, c='blue', marker='o')
    ax.axhline(color='black')
    ax.axvline(color='black')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-1, 1)
    plt.xlabel('Ay')
    plt.ylabel('Ax')
    plt.title('GG Plot')
    return fig

# plot_GG(Ay, Ax)


# In[16]:


# Writing HTML
def write_HTML(notes_df):
    # Title
    html="<!DOCTYPE html><html>"
    html = write_notes(notes_df, html)
    html = HTML_broken_sensors(html, broken_sensors)
    html = HTML_damper_histograms(html)
    html = HTML_roll_gradient(html)
    html = HTML_LLT(html)
    html = HTML_GG(html)
    html += "</html>"
    return html

def write_notes(notes_df, html_string):
    # Title
    html_string += "<head><h1>{}-{}-{}</h1></head><hr/>".format(notes_df.iloc[1,1], notes_df.iloc[2,1], notes_df.iloc[5,1])
    
    #Details
    html_string += "<p>"
    html_string += "<b>Venue:</b> {}<br/><b>Device:</b> {}<br/><b>Comments:</b> {}<br/>".format(notes_df.iloc[0,1], notes_df.iloc[3,1], notes_df.iloc[4,1])
    html_string += "<b>Log Time:</b> {}<br/><b>Sample Rate:</b> {} {}<br/>".format(notes_df.iloc[6,1], notes_df.iloc[7,1], notes_df.iloc[7,2])
    html_string += "<b>Duration:</b> {} {}<br/><b>Range:</b> {}<br/><b>Beacon Markers:</b> {}<br/>".format(notes_df.iloc[8,1], notes_df.iloc[8,2], notes_df.iloc[9,1], notes_df.iloc[10,1])
    html_string += "<b>Session:</b> {}<br/>".format(notes_df.iloc[4,4])
    html_string += "<b>Origin Time:</b> {} {}<br/><b>Start Time:</b> {} {}<br/>".format(notes_df.iloc[5,4], notes_df.iloc[5,5], notes_df.iloc[6,4], notes_df.iloc[6,5])
    html_string += "<b>End Time:</b> {} {}<br/>".format(notes_df.iloc[7,4], notes_df.iloc[7,5])
    html_string += "<b>Start Distance:</b> {} {}<br/><b>End Distance:</b> {} {}<br/>".format(notes_df.iloc[8,4], notes_df.iloc[8,5], notes_df.iloc[9,4], notes_df.iloc[9,5])
    html_string += "</p>"
    
    return html_string

def HTML_broken_sensors(html_string, broken_sensors):
    if (len(broken_sensors) > 0):
        html_string += "<br/><font color=\"red\"><b>Warning: Possible Broken Sensors</b></font></br>"
        for item in broken_sensors:
            html_string += item + ", "
        html_string = html_string[:len(html_string)-2]
    return html_string
    
def HTML_damper_histograms(html_string):
    # Add section title
    html_string += "<h2>Damper Velocity Histograms</h2><hr/>"
    html_string += "<h3>Non-gated</h3>"
    
    # Add Damper velocity histograms (non-gated)
    f1, f2, f3, f4 = generate_damper_histograms(cleaned_df)
    s1 = convert_fig_to_html(f1)
    s2 = convert_fig_to_html(f2)
    s3 = convert_fig_to_html(f3)
    s4 = convert_fig_to_html(f4)
    
    html_string += s1
    html_string += s2
    html_string += s3
    html_string += s4
    
    html_string += "<h3>Gated at 1G</h3>"
    # Add Damper velocity histograms (non-gated)
    f1, f2, f3, f4 = generate_damper_histograms(gating(cleaned_df, "G Force Lat", 1))
    s1 = convert_fig_to_html(f1)
    s2 = convert_fig_to_html(f2)
    s3 = convert_fig_to_html(f3)
    s4 = convert_fig_to_html(f4)
    
    html_string += s1
    html_string += s2
    html_string += s3
    html_string += s4
    
    return html_string

def HTML_roll_gradient(html_string):
    ng_str, g_str = roll_gradient_HTML()
    # Add section title
    html_string += "<h2>Roll Gradient</h2><hr/>"
    html_string += "<h3>Non-gated</h3><br/>"
    html_string += ng_str
    html_string += "<h3>Gated at 1G</h3><br/>"
    html_string += g_str
    
    return html_string

def HTML_roll_gradient(html_string):
    ng_str, g_str = roll_gradient_HTML()
    # Add section title
    html_string += "<h2>Roll Gradient</h2><hr/>"
    html_string += "<h3>Non-gated</h3><br/>"
    html_string += ng_str
    html_string += "<h3>Gated at 1G</h3><br/>"
    html_string += g_str
    
    return html_string

def HTML_LLT(html_string):
    html_string += "<h2>Lateral Load Transfer</h2><hr/>"
    fig, string = plot_LLT(Ay, cleaned_df["Time"])
    s1 = convert_fig_to_html(fig)
    html_string += string
    html_string += s1
    
    return html_string

def HTML_GG(html_string):
    html_string += "<h2>GG Plot</h2><hr/>"
    fig = plot_GG(Ay, Ax)
    s1 = convert_fig_to_html(fig)
    html_string += s1
    
    return html_string


# In[17]:


# Creating PDF
def create_PDF(string):
    import pdfkit
    pdfkit.from_string(string, r'out.pdf')

create_PDF(write_HTML(notes))


# In[ ]:




