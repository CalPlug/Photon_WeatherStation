import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { DatagraphComponent } from './datagraph.component';

describe('DatagraphComponent', () => {
  let component: DatagraphComponent;
  let fixture: ComponentFixture<DatagraphComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ DatagraphComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(DatagraphComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
