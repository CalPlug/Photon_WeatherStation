import { Component, OnInit, AfterViewInit, HostBinding, Input } from '@angular/core';
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { Http, Response, Headers, RequestOptions } from '@angular/http';
import { Observable } from 'rxjs/Observable';
import { Category } from '../category';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

@Component({
  selector: 'ssmap',
  templateUrl: './newdtp.component.html',
  styleUrls: ['./newdtp.component.css']
})
export class NewdtpComponent implements OnInit {
  
  
  display:boolean = false;
  Cindex:number = 0;
  Wdirct:number = 0;
  buttondis:string="show image";

  @Input() message_test: string;


  //wind direction and wind speed is over here
    // Radar
    Cleardata=[0,0,0,0,0,0,0,0];
    public radarChartLabels:string[] = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W','NW'];
 
    public radarChartData:any = [
      {data: [0,0,0,0,0,0,10,0],label:null},
      
  
      
    ];
    public radarChartType:string = 'radar';
   
    // events
    public chartClicked(e:any):void {
      console.log(e);
    }
   
    public chartHovered(e:any):void {
      console.log(e);
    }
  
// end of radar

  public Winddirsp(){
    let degree = this.mostRecent["WindDirection"];
    let speed = this.mostRecent["WindSpeed"];
    let minor_speed = speed-5;

    
    
    if (degree==0 || degree==360){
      this.radarChartData = [      
        {
          data: [speed,minor_speed,minor_speed,minor_speed,minor_speed,minor_speed,minor_speed,minor_speed], label: 'Wind direction & speed'
        }
      ];
    }else if(degree>0 && degree<90){

      this.radarChartData = [      
        {
          data: [minor_speed,speed,minor_speed,minor_speed,minor_speed,minor_speed,minor_speed,minor_speed], label: 'Wind direction & speed'
        }
      ];

    }else if(degree==90){

      this.radarChartData = [      
        {
          data: [minor_speed,minor_speed,speed,minor_speed,minor_speed,minor_speed,minor_speed,minor_speed], label: 'Wind direction & speed'
        }
      ];

    }else if(degree>90 && degree<180){

      this.radarChartData = [      
        {
          data: [minor_speed,minor_speed,minor_speed,speed,minor_speed,minor_speed,minor_speed,minor_speed], label: 'Wind direction & speed'
        }
      ];

    }else if(degree==180){

      this.radarChartData = [      
        {
          data: [minor_speed,minor_speed,minor_speed,minor_speed,speed,minor_speed,minor_speed,minor_speed], label: 'Wind direction & speed'
        }
      ];

    }else if(degree>180 && degree<270){

      this.radarChartData = [      
        {
          data: [minor_speed,minor_speed,minor_speed,minor_speed,minor_speed,speed,minor_speed,minor_speed], label: 'Wind direction & speed'
        }
      ];

    }else if(degree==270){

      this.radarChartData = [      
        {
          data: [minor_speed,minor_speed,minor_speed,minor_speed,minor_speed,minor_speed,speed,minor_speed], label: 'Wind direction & speed'
        }
      ];

    }else if(degree>270){

      this.radarChartData = [      
        {
          data: [minor_speed,minor_speed,minor_speed,minor_speed,minor_speed,minor_speed,minor_speed,speed], label: 'Wind direction & speed'
        }
      ];

    }




  }



  public subname:string = "";

  
  private apiUrl = 'api/';

  categories:Category[] = [];
  TheTile:string = "";
  public categoriesObj={};
  dataready:boolean = false;

  public dyOptions = {width: 'auto', animatedZooms:true, pointSize: 4, labels:["Time",''] }
  public dyCategoryData = {};

    

  constructor(private http : HttpClient) { 
    
  }

  public mostRecent = {};



  ngOnInit() {
      this.getCategories(data => this.populateCategories(data));
      
  }

  ngAfterViewInit() {
      // console.log(this.categoryData);
  }

  getCategories(onSuccess?) {
      this.http.get(this.apiUrl+'categories').subscribe(data => {
          var catData:string[] = data['categories'];
          var lenCats:number = catData.length;
          for (var i:number = 0; i<lenCats; ++i){
              var cat:string = catData[i];

              var parent:string = cat.split('_',1)[0];

              if (parent) {
                  
                  if (!(parent in this.categoriesObj))
                      this.categoriesObj[parent] = new Category(parent);

                  if (cat != parent){
                      var index = cat.indexOf('_');
                      var child = cat.substring(index+1);
                      this.categoriesObj[parent].children.push(child);
                  }
              }
          }
          this.categories = Object.values(this.categoriesObj);
          if (typeof(onSuccess) !== 'undefined') onSuccess(this.categories);
      });
  }

  getCategoryData(category, callback){
      this.http.get(this.apiUrl+'data/'+ encodeURIComponent(category)).subscribe((data) => callback(data['data']));
  }

  populateCategories(categories){
      const lenCats:number = this.categories.length;
      for (var i:number = 0; i<lenCats; ++i){
          const lenChildren = this.categories[i].children.length;
          const catName = this.categories[i].name;
          console.log('derp');
          if (!lenChildren){
              this.getCategoryData(this.categories[i].name, data => this.populateSingle(this, data, catName));
          } else {
              for(var j:number = 0; j<lenChildren; ++j){
                  if (!j){
                      const childName = this.categories[i].children[j];
                      this.getCategoryData(this.categories[i].name + '_' + this.categories[i].children[j], data => this.populateMultiple(this, data, catName,childName ));
                      break;
                  }
              }
          }
      }
      this.dataready = true;
      this.display = true;
      
  }

  private populateSingle(ref, data, categoryName){
      const lenData = data.length;
      var dyArrData:Array<any> = [];
      for (var j:number = 0; j<data.length; ++j){

          dyArrData.push(
              [
                  new Date(data[j].utcTime),
                  parseFloat(data[j][categoryName]) || 0
              ]
          )
          ref.mostRecent[categoryName] = parseFloat(data[j][categoryName]) || 0;
      }

      ref.dyCategoryData[categoryName] = dyArrData;

  }

  private populateMultiple(ref, data, categoryName, childName){
      const lenData = data.length;
      var dyArrData:Array<any> = [];
      for (var j:number = 0; j<data.length; ++j){
          dyArrData.push(
              [
                  new Date(data[j].utcTime),
                  parseFloat( data[j][categoryName+"_"+childName])
              ]
          )
          ref.mostRecent[categoryName] = parseFloat(data[j][categoryName+"_"+childName]) || 0;
      }
          
      ref.dyCategoryData[categoryName] = dyArrData;

      // console.log(ref.dyCategoryData);
  }

  chartSelect(mainCat, subCat){
      this.getCategoryData(mainCat+ '_' + subCat, data => this.populateMultiple(this, data, mainCat,subCat )); 
  }

  checkDefined(category){
      return typeof(this.dyCategoryData[category]) !== 'undefined';
  }

}