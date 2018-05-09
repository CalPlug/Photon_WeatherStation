import { Component, OnInit, AfterViewInit } from '@angular/core';
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { Http, Response, Headers, RequestOptions } from '@angular/http';
import { Observable } from 'rxjs/Observable';
import { Category } from '../category';

@Component({
  selector: 'ssmap',
  template: `
  <div class="separate">
    <div class="ops">
        <div *ngFor="let category of categories" id="cat{{category.name}}" class="catView">
          
        </div>

        <div>


          <button class="button" type="button" 
          (click)="clicked = true" 
          (click)="nextCate()"
          (click)="showCate()">
          Next Category
          </button>


          <h1>{{TheTile}}</h1>
          <ng-dygraphs
            [data]="dyCategoryData[TheTile]"
            [options]="{width: 'auto', animatedZooms: true, pointSize: 4, labels:['Time', TheTile], xlabel:'Time', ylabel:TheTile}">
          </ng-dygraphs>  
        </div>

    </div>
  </div>
  
  
  
  `,
  styleUrls: ['./newdtp.component.css']
})
export class NewdtpComponent implements OnInit {
  
  
  display:boolean = false;
  index:number = 0;
  
  
  
  nextCate(){
    if(this.index>=(this.categories.length - 1)){
      this.index = 0;
    }else{
      this.index+=1;
    }
  }

  showCate(){
    if(this.display){
      this.display=false;
    }else{
    this.display = true;
    }
    this.TheTile = this.categories[this.index].name;
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

  ngOnInit() {
    this.getCategories(data => this.populateCategories(data));
  }

  ngAfterViewInit(){
    
  }

  getCategories(onSuccess?){
    this.http.get(this.apiUrl+'categories').subscribe(data =>{
      let catData:string[] = data['categories'];
      let lenCats:number = catData.length;
      for(var i:number = 0; i<lenCats; ++i){
        let cat:string = catData[i];

        let parent:string = cat.split('_',1)[0];

        if(parent) {
          
          if(!(parent in this.categoriesObj))
              this.categoriesObj[parent] = new Category(parent);
          if(cat != parent){
            let index = cat.indexOf('_');
            let child = cat.substring(index+1);
            this.categoriesObj[parent].children.push(child);
          }
        }
      }
      this.categories = Object.values(this.categoriesObj);
      if (typeof(onSuccess) !== 'undefined')onSuccess(this.categories);
    })
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
