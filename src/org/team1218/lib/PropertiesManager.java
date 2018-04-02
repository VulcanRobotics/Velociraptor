package org.team1218.lib;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Properties;

public class PropertiesManager {
	Properties properties;
	File propertiesFile;
	boolean loaded = false;
	
	public PropertiesManager(File file) {
		this.propertiesFile = file;
		properties = new Properties();
	}
	
	public PropertiesManager(String path) {
		this(new File(path));
	}
	
	public boolean load() {
			FileInputStream properiesStream;
			try {
				properiesStream = new FileInputStream(propertiesFile);
				properties.load(properiesStream);
				loaded = true;
				return true;
			} catch (FileNotFoundException e) {
				System.out.println("Load Failed: File not found");
				e.printStackTrace();
				return false;
			} catch (IOException e) {
				System.out.println("Load Failed: IO Exception has occured");
				e.printStackTrace();
				return false;
			}
	}
	
	public int getInt(String key) {
		return getInt(key,0);
	}
	
	public int getInt(String key, int defaultReturn) {
		if(loaded) {
			String property = properties.getProperty(key);
			try {
				if(property != null) {
					return Integer.parseInt(property);
				}else {
					return defaultReturn;
				}
			}catch(Exception e) {
				return defaultReturn;
			}
		}else {
			return defaultReturn;
		}
	}
	
	public double getDouble(String key) {
		return getDouble(key,0);
	}
	
	public double getDouble(String key, double defaultReturn) {
		if(loaded) {
			String property = properties.getProperty(key);
			try {
				if(property != null) {
					return Double.parseDouble(property);
				}else {
					return defaultReturn;
				}
			}catch(Exception e) {
				return defaultReturn;
			}
		}else {
			return defaultReturn;
		}
	}
	
	public boolean getBoolean(String key) {
		return getBoolean(key,false);
	}
	
	public boolean getBoolean(String key, boolean defaultReturn) {
		if(loaded) {
			String property = properties.getProperty(key);
			try {
				if(property != null) {
					return Boolean.parseBoolean(property);
				}else {
					return defaultReturn;
				}
			}catch(Exception e) {
				return defaultReturn;
			}
		}else {
			return defaultReturn;
		}
	}
	
	public int[] getInts(String key) {
		return getInts(key, new int[0]);
	}
	
	public int[] getInts(String key,int[] defaultReturn) {
		if(loaded) {
			try {
				String property = properties.getProperty(key);
				if(property != null) {
					String[] stringArray = property.split(",");
					int[] intArray = new int[stringArray.length];
					for(int i = 0; i < stringArray.length; i++) {
						intArray[i] = Integer.parseInt(stringArray[i]);
					}
					return intArray;
				}else {
					return defaultReturn;
				}
				
			}catch(Exception E) {
				return defaultReturn;
			}
		}else {
			return defaultReturn;
		}
	}
	
	public double[] getDoubles(String key) {
		return getDoubles(key, new double[0]);
	}
	
	public double[] getDoubles(String key,double[] defaultReturn) {
		if(loaded) {
			try {
				String property = properties.getProperty(key);
				if(property != null) {
					String[] stringArray = property.split(",");
					double[] doubleArray = new double[stringArray.length];
					for(int i = 0; i < stringArray.length; i++) {
						doubleArray[i] = Double.parseDouble(stringArray[i]);
					}
					return doubleArray;
				}else {
					return defaultReturn;
				}
				
			}catch(Exception E) {
				return defaultReturn;
			}
		}else {
			return defaultReturn;
		}
	}
	
	
	public boolean[] getBooleans(String key) {
		return getBooleans(key, new boolean[0]);
	}
	
	public boolean[] getBooleans(String key,boolean[] defaultReturn) {
		if(loaded) {
			try {
				String property = properties.getProperty(key);
				if(property != null) {
					String[] stringArray = property.split(",");
					boolean[] booleanArray = new boolean[stringArray.length];
					for(int i = 0; i < stringArray.length; i++) {
						booleanArray[i] = Boolean.parseBoolean(stringArray[i]);
					}
					return booleanArray;
				}else {
					return defaultReturn;
				}
				
			}catch(Exception E) {
				return defaultReturn;
			}
		}else {
			return defaultReturn;
		}
	}
}
